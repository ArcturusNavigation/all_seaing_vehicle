#include "all_seaing_vehicle/obstacle_bbox_overlay.hpp"
#include <iostream>

void ObstacleBboxOverlay::obstacle_bbox_fusion_cb(
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg)
{
    // Get transform from LiDAR to camera
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_bbox_msg->header.frame_id, in_map_msg->header.frame_id);

    // Match clusters and bounding boxes
    all_seaing_interfaces::msg::ObstacleMap new_map;
    new_map.header = in_map_msg->header;
    std::unordered_set<int> chosen_indices;
    for (const all_seaing_interfaces::msg::Obstacle &obstacle : in_map_msg->obstacles)
    {
        // Transform from LiDAR to camera coordinate systems
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = obstacle.local_point.x;
        lidar_point.y = obstacle.local_point.y;
        lidar_point.z = obstacle.local_point.z;
        geometry_msgs::msg::Point tf_point;
        tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, tf_point, m_pc_cam_tf);

        // Project 3D point onto the image plane using the intrinsic matrix.
        // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
        cv::Point2d xy_rect = m_cam_model.project3dToPixel(cv::Point3d(tf_point.y, tf_point.z, -tf_point.x));

        // Match clusters if within bounds and in front of the boat
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) &&
            (xy_rect.y >= 0) && (xy_rect.y < m_cam_model.cameraInfo().height) && (obstacle.local_point.x >= 0))
        {

            // RCLCPP_INFO(this->get_logger(), "Projected 2D point: x: %f, y: %f", xy_rect.x, xy_rect.y);

            // Iterate through bounding boxes
            double best_dist = 1e9;
            int best_match = -1;
            for (unsigned long i = 0; i < in_bbox_msg->boxes.size(); i++)
            {
                // Skip indices already chosen
                if (chosen_indices.find(i) != chosen_indices.end())
                    continue;

                const all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = in_bbox_msg->boxes[i];

                double center_x = (bbox.max_x + bbox.min_x) / 2;
                double center_y = (bbox.max_y + bbox.min_y) / 2;

                double curr_dist = std::hypot(center_x - xy_rect.x, center_y - xy_rect.y);
                if (curr_dist < best_dist)
                {
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
    if (m_viz)
        markers(new_map);
}

geometry_msgs::msg::TransformStamped ObstacleBboxOverlay::get_tf(const std::string &in_target_frame,
                                                                 const std::string &in_src_frame)
{
    geometry_msgs::msg::TransformStamped tf;
    m_pc_cam_tf_ok = false;
    try
    {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        m_pc_cam_tf_ok = true;
        RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good");
        RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s", in_target_frame.c_str(), in_src_frame.c_str());
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return tf;
}

void ObstacleBboxOverlay::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg)
{
    m_cam_model.fromCameraInfo(info_msg);
}

void ObstacleBboxOverlay::markers(const all_seaing_interfaces::msg::ObstacleMap &in_map_msg)
{
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray text_marker_array;

    for (const auto &obstacle : in_map_msg.obstacles)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "odom";
        marker.ns = "labeled_obstacle";
        marker.id = obstacle.id;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;
        marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        marker.pose.position.x = obstacle.global_point.x;
        marker.pose.position.y = obstacle.global_point.y;
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.header.frame_id = "odom";
        text_marker.ns = "labeled_text";
        text_marker.id = obstacle.id;
        text_marker.scale.z = 0.7; // Text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        text_marker.text = std::to_string(obstacle.label);
        text_marker.pose.position.x = obstacle.global_point.x;
        text_marker.pose.position.y = obstacle.global_point.y;
        text_marker.pose.position.z = 1.0;
        text_marker_array.markers.push_back(text_marker);
    }

    m_marker_array_pub->publish(marker_array);
    m_text_marker_array_pub->publish(text_marker_array);

}

ObstacleBboxOverlay::ObstacleBboxOverlay() : Node("obstacle_bbox_overlay")
{
    // Initialize parameters
    this->declare_parameter<bool>("viz", true);
    m_viz = this->get_parameter("viz").as_bool();

    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/img_info_src",
        1,
        std::bind(&ObstacleBboxOverlay::intrinsics_cb, this, std::placeholders::_1));
    m_bbox_sub.subscribe(this, "/bounding_boxes", rmw_qos_profile_system_default);
    m_map_sub.subscribe(this, "/unlabeled_map", rmw_qos_profile_system_default);

    // Publisher
    m_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("/labeled_map", 1);
    m_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/labeled_chull_markers", 10);
    m_text_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/labeled_text_markers", 10);

    // Send cluster msg and bbox msg to ClusterBboxFusionCb
    m_obstacle_bbox_sync = std::make_shared<ObstacleBboxSync>(ObstacleBboxPolicy(10), m_bbox_sub, m_map_sub);
    m_obstacle_bbox_sync->registerCallback(std::bind(&ObstacleBboxOverlay::obstacle_bbox_fusion_cb, this, std::placeholders::_1, std::placeholders::_2));
}

ObstacleBboxOverlay::~ObstacleBboxOverlay() {}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleBboxOverlay>());
    rclcpp::shutdown();
    return 0;
}
