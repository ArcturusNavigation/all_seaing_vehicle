#include "all_seaing_perception/obstacle_bbox_overlay.hpp"

#include <unordered_set>

#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

void ObstacleBboxOverlay::obstacle_bbox_fusion_cb(
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg) {

    // Get transform from LiDAR to camera
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_bbox_msg->header.frame_id, in_map_msg->local_header.frame_id);

    // Match clusters and bounding boxes
    all_seaing_interfaces::msg::ObstacleMap new_map;
    new_map.local_header = in_map_msg->local_header;
    new_map.global_header = in_map_msg->global_header;
    std::unordered_set<int> chosen_indices;
    for (const all_seaing_interfaces::msg::Obstacle &obstacle : in_map_msg->obstacles) {

        // Transform from LiDAR to camera coordinate systems
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = obstacle.local_point.point.x;
        lidar_point.y = obstacle.local_point.point.y;
        lidar_point.z = obstacle.local_point.point.z;
        geometry_msgs::msg::Point camera_point;
        tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, camera_point, m_pc_cam_tf);

        // Project 3D point onto the image plane using the intrinsic matrix.
        // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
        cv::Point2d xy_rect = m_cam_model.project3dToPixel(
            cv::Point3d(camera_point.y, camera_point.z, -camera_point.x));

        // Match clusters if within bounds and in front of the boat
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
            (xy_rect.y < m_cam_model.cameraInfo().height) && (obstacle.local_point.point.x >= 0)) {

            // Iterate through bounding boxes
            double best_dist = 1e9;
            int best_match = -1;
            for (unsigned long i = 0; i < in_bbox_msg->boxes.size(); i++) {
                // Skip indices already chosen
                if (chosen_indices.find(i) != chosen_indices.end())
                    continue;

                const all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = in_bbox_msg->boxes[i];

                double center_x = (bbox.max_x + bbox.min_x) / 2;
                double center_y = (bbox.max_y + bbox.min_y) / 2;

                double curr_dist = std::hypot(center_x - xy_rect.x, center_y - xy_rect.y);
                if (curr_dist < best_dist) {
                    best_match = i;
                    best_dist = curr_dist;
                }
            }

            // If best_match was never assigned, then skip
            if (best_match == -1)
                continue;

            // Add best match index to chosen indices and add label to cluster
            all_seaing_interfaces::msg::Obstacle new_obstacle = obstacle;
            new_obstacle.label = in_bbox_msg->boxes[best_match].label;
            new_map.obstacles.push_back(new_obstacle);
            chosen_indices.insert(best_match);
        }
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
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10,
        std::bind(&ObstacleBboxOverlay::intrinsics_cb, this, std::placeholders::_1));
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_default);
    m_map_sub.subscribe(this, "obstacle_map/unlabeled", rmw_qos_profile_default);

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
