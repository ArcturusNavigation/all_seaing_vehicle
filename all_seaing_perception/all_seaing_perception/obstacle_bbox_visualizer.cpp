#include <iostream>
#include <fstream>

#include "all_seaing_perception/obstacle_bbox_visualizer.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h" 



ObstacleBboxVisualizer::ObstacleBboxVisualizer() : Node("obstacle_bbox_visualizer") {

    // initialize transform listener
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Initialize subscribers
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10,
        std::bind(&ObstacleBboxVisualizer::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "image", rmw_qos_profile_sensor_data);
    m_obstacle_map_sub.subscribe(this, "obstacle_map/labeled", rmw_qos_profile_default);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_default);

    // Initialize publisher
    m_image_pub = this->create_publisher<sensor_msgs::msg::Image>("image/obstacle_visualized", 10);

    // Initialize synchronizer
    m_obstacle_bbox_visualizer_sync = std::make_shared<ObstacleBboxVisualizerSync>(ObstacleBboxVisualizerPolicy(10), m_image_sub, m_obstacle_map_sub, m_bbox_sub);
    m_obstacle_bbox_visualizer_sync->registerCallback(std::bind(&ObstacleBboxVisualizer::image_obstacle_cb, this,
                                       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
                                        
    this->declare_parameter("color_label_mappings_file", "");
    color_label_mappings_file = this->get_parameter("color_label_mappings_file").as_string();
    std::ifstream yamlFile(color_label_mappings_file);

    if (yamlFile.is_open()) {
        config_yaml = YAML::Load(yamlFile);  
        for (YAML::const_iterator it = config_yaml.begin(); it != config_yaml.end(); ++it) {
            // RCLCPP_INFO(this->get_logger(), "label %d and color %s", it->second.as<int>(), it->second.as<std::string>().c_str());
            label_color_map[it->second.as<int>()] = it->first.as<std::string>();
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", color_label_mappings_file.c_str());
    }

    yamlFile.close();

}

ObstacleBboxVisualizer::~ObstacleBboxVisualizer() {}
void ObstacleBboxVisualizer::image_obstacle_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& in_img_msg,
    const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr& in_map_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr& in_bbox_msg) {

    // Get the transform from LiDAR to camera
    if (!m_pc_cam_tf_ok) {
        m_pc_cam_tf = get_tf(in_bbox_msg->header.frame_id, in_map_msg->local_header.frame_id);
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    for (const auto& obstacle : in_map_msg->obstacles) {

        // transform the lidar point to camera frame
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = obstacle.local_point.point.x;
        lidar_point.y = obstacle.local_point.point.y;
        lidar_point.z = obstacle.local_point.point.z;
        geometry_msgs::msg::Point camera_point;
        tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, camera_point, m_pc_cam_tf);

        // RCLCPP_INFO(this->get_logger(), "Lidar point: (%f, %f, %f)", lidar_point.x, lidar_point.y, lidar_point.z);
        // RCLCPP_INFO(this->get_logger(), "Camera point: (%f, %f, %f)", camera_point.x, camera_point.y, camera_point.z);

        // find the centroid and display it.
        cv::Point3d centroid(camera_point.y,
                             camera_point.z,
                             -camera_point.x);
        cv::Point2d pixel_centroid = m_cam_model.project3dToPixel(centroid);


        if (pixel_centroid.x >= 0 && pixel_centroid.x < cv_ptr->image.cols &&
            pixel_centroid.y >= 0 && pixel_centroid.y < cv_ptr->image.rows) {
            cv::Scalar color = get_color_for_label(obstacle.label);
            
            cv::circle(cv_ptr->image, pixel_centroid, 5, color, -1);
        } 
        else { 
            RCLCPP_WARN(this->get_logger(), "Centroid outside image bounds: (%f, %f)", pixel_centroid.x, pixel_centroid.y);
        }

        // display bbox -> 2d bbox
        geometry_msgs::msg::Point camera_bbox_min;
        geometry_msgs::msg::Point camera_bbox_max;
        tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_min, camera_bbox_min, m_pc_cam_tf);
        tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_max, camera_bbox_max, m_pc_cam_tf);
        cv::Point3d ptmin(camera_bbox_min.y, camera_bbox_min.z, -camera_bbox_min.x);
        cv::Point3d ptmax(camera_bbox_max.y, camera_bbox_max.z, -camera_bbox_max.x);
        cv::Point2d pt1 = m_cam_model.project3dToPixel(ptmin);
        cv::Point2d pt2 = m_cam_model.project3dToPixel(ptmax);
        cv::Scalar color(0, 255, 255);
        cv::rectangle(cv_ptr->image, pt1, pt2, color, 2);

        // RCLCPP_INFO(this->get_logger(), "3d lidar point: (%f, %f, %f)", lidar_point.x, lidar_point.y, lidar_point.z);
        // RCLCPP_INFO(this->get_logger(), "3d bbox min point: (%f, %f, %f)", lidar_bbox_min.x, lidar_bbox_min.y, lidar_bbox_min.z);
        // RCLCPP_INFO(this->get_logger(), "3d camera point: (%f, %f, %f)", centroid.x, centroid.y, centroid.z);
        // RCLCPP_INFO(this->get_logger(), "3d Bbox min: (%f, %f, %f)", ptmin.x, ptmin.y, ptmin.z);
        // RCLCPP_INFO(this->get_logger(), "camera point: (%f, %f)", pixel_centroid.x, pixel_centroid.y);
        // RCLCPP_INFO(this->get_logger(), "Bbox min: (%f, %f)", pt1.x, pt1.y);
    }

    // draw the bounding boxes
    for (const auto& bbox : in_bbox_msg->boxes) {
        cv::Scalar color = get_color_for_label(bbox.label);
        cv::Point pt1(bbox.min_x, bbox.min_y);
        cv::Point pt2(bbox.max_x, bbox.max_y);
        cv::rectangle(cv_ptr->image, pt1, pt2, color, 2);
    }


    m_image_pub->publish(*cv_ptr->toImageMsg());
}

void ObstacleBboxVisualizer::intrinsics_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

geometry_msgs::msg::TransformStamped ObstacleBboxVisualizer::get_tf(const std::string& in_target_frame,
                                                                    const std::string& in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    m_pc_cam_tf_ok = false;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        m_pc_cam_tf_ok = true;
        RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good in visualizer");
        RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s",
                    in_target_frame.c_str(), in_src_frame.c_str());
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Error in LiDAR to Camera TF in visualizer: %s", ex.what());
    }
    return tf;
}

cv::Scalar ObstacleBboxVisualizer::get_color_for_label(const int& label) {
    std::string color = label_color_map[label];

    if (color == "orange") return cv::Scalar(0, 165, 255);
    if (color == "red") return cv::Scalar(0, 0, 255);
    if (color == "green") return cv::Scalar(0, 255, 0);
    if (color == "black") return cv::Scalar(0, 0, 0);
    if (color == "white") return cv::Scalar(255, 255, 255);
    return cv::Scalar(255, 0, 0);  // blue
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleBboxVisualizer>());
    rclcpp::shutdown();
    return 0;
}