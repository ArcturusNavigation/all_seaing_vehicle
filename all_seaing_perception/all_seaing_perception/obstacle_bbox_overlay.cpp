#include "all_seaing_perception/obstacle_bbox_overlay.hpp"

#include <unordered_set>

#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Check if a point is in bounds
bool in_bounds(const cv::Point2d &point, sensor_msgs::msg::CameraInfo camera_info) {
    return point.x >= 0 && point.x < camera_info.width && point.y >= 0 &&
           point.y < camera_info.height;
}

int ObstacleBboxOverlay::get_matching_obstacle_iou(
    const all_seaing_interfaces::msg::Obstacle &obstacle,
    const std::unordered_set<int> &chosen_indices,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {

    geometry_msgs::msg::Point bbox_min_cam, bbox_max_cam;
    tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_min, bbox_min_cam, m_pc_cam_tf);
    tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_max, bbox_max_cam, m_pc_cam_tf);

    cv::Point2d bbox1_xy = m_is_sim
                               ? all_seaing_perception::project3dToPixel(m_cam_model,
                                     cv::Point3d(bbox_min_cam.y, bbox_min_cam.z, -bbox_min_cam.x))
                               : all_seaing_perception::project3dToPixel(m_cam_model,
                                     cv::Point3d(bbox_min_cam.x, bbox_min_cam.y, bbox_min_cam.z));
    cv::Point2d bbox2_xy = m_is_sim
                               ? all_seaing_perception::project3dToPixel(m_cam_model,
                                     cv::Point3d(bbox_max_cam.y, bbox_max_cam.z, -bbox_max_cam.x))
                               : all_seaing_perception::project3dToPixel(m_cam_model,
                                     cv::Point3d(bbox_max_cam.x, bbox_max_cam.y, bbox_max_cam.z));

    // mins / max become strange due to coordinate system. need to recalculate
    cv::Point bbox_min_2d(std::min(bbox1_xy.x, bbox2_xy.x), std::min(bbox1_xy.y, bbox2_xy.y));
    cv::Point bbox_max_2d(std::max(bbox1_xy.x, bbox2_xy.x), std::max(bbox1_xy.y, bbox2_xy.y));

    // Match clusters if within bounds and in front of the boat
    if (in_bounds(bbox_min_2d, m_cam_model.cameraInfo()) && 
        in_bounds(bbox_max_2d, m_cam_model.cameraInfo()) &&
        (m_is_sim ? bbox_min_cam.x >= 0 : bbox_min_cam.z >= 0)) {

        // Iterate through bounding boxes
        double best_iou = 0;
        int best_match = -1;
        for (unsigned long i = 0; i < in_bbox_msg->boxes.size(); i++) {
            // Skip indices already chosen
            if (chosen_indices.find(i) != chosen_indices.end())
                continue;

            const all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = in_bbox_msg->boxes[i];

            cv::Point camera_bbox_min(bbox.min_x, bbox.min_y);
            cv::Point camera_bbox_max(bbox.max_x, bbox.max_y);

            // Calculate intersection over union
            // TODO: parameterize these margins (also make the variable names more clear), or remove them?
            cv::Rect2d bbox1(bbox_min_2d.x - 1, bbox_min_2d.y - 1, bbox_max_2d.x - bbox_min_2d.x + 2,
                             bbox_max_2d.y - bbox_min_2d.y + 2); // LiDAR cluster bbox (what are those margins?)
            // TODO: Add padding to camera bounding box before checking the IoU
            cv::Rect2d bbox2(camera_bbox_min.x, camera_bbox_min.y,
                             camera_bbox_max.x - camera_bbox_min.x,
                             camera_bbox_max.y - camera_bbox_min.y); // Camera detection bbox

            cv::Rect2d intersection = bbox1 & bbox2;
            cv::Rect2d union_rect = bbox1 | bbox2;

            double iou = intersection.area() / union_rect.area();

            if (iou > best_iou) {
                best_match = i;
                best_iou = iou;
            }
        }

        return best_match;
    }
    return -1;
}

void ObstacleBboxOverlay::obstacle_bbox_fusion_cb(
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg) {

    // Get transform from LiDAR to camera
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_bbox_msg->header.frame_id, in_map_msg->local_header.frame_id);

    // Match clusters and bounding boxes
    all_seaing_interfaces::msg::ObstacleMap new_map;
    new_map.ns = "labeled";
    new_map.local_header = in_map_msg->local_header;
    new_map.header = in_map_msg->header;
    new_map.is_labeled = true;
    std::unordered_set<int> chosen_indices;

    for (const all_seaing_interfaces::msg::Obstacle &obstacle : in_map_msg->obstacles) {
        int best_match = get_matching_obstacle_iou(obstacle, chosen_indices, in_bbox_msg);

        // If best_match was never assigned, then skip
        if (best_match == -1)
            continue;

        // Add best match index to chosen indices and add label to cluster
        all_seaing_interfaces::msg::Obstacle new_obstacle = obstacle;
        new_obstacle.label = in_bbox_msg->boxes[best_match].label;
        new_map.obstacles.push_back(new_obstacle);
        chosen_indices.insert(best_match);
    }
    m_map_pub->publish(new_map);
}

geometry_msgs::msg::TransformStamped ObstacleBboxOverlay::get_tf(const std::string &in_target_frame,
                                                                 const std::string &in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    m_pc_cam_tf_ok = false;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        m_pc_cam_tf_ok = true;
        RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good");
        RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s",
                    in_target_frame.c_str(), in_src_frame.c_str());
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return tf;
}

void ObstacleBboxOverlay::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

ObstacleBboxOverlay::ObstacleBboxOverlay() : Node("obstacle_bbox_overlay") {
    // Initialize parameters
    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10,
        std::bind(&ObstacleBboxOverlay::intrinsics_cb, this, std::placeholders::_1));
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_default);
    m_map_sub.subscribe(this, "obstacle_map/raw", rmw_qos_profile_default);

    // Publisher
    m_map_pub =
        this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/labeled", 10);

    // Send cluster msg and bbox msg to ClusterBboxFusionCb
    m_obstacle_bbox_sync =
        std::make_shared<ObstacleBboxSync>(ObstacleBboxPolicy(10), m_bbox_sub, m_map_sub);
    m_obstacle_bbox_sync->registerCallback(std::bind(&ObstacleBboxOverlay::obstacle_bbox_fusion_cb,
                                                     this, std::placeholders::_1,
                                                     std::placeholders::_2));
}

ObstacleBboxOverlay::~ObstacleBboxOverlay() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleBboxOverlay>());
    rclcpp::shutdown();
    return 0;
}
