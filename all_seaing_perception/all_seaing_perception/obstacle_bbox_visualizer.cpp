#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "yaml-cpp/yaml.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <string>
#include <vector>

class ObstacleBboxVisualizer : public rclcpp::Node {
public:
    ObstacleBboxVisualizer() : Node("obstacle_bbox_visualizer") {
        // Initialize parameters
        this->declare_parameter<bool>("is_sim", false);
        m_is_sim = this->get_parameter("is_sim").as_bool();

        // Initialize transform listener
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
        m_image_pub =
            this->create_publisher<sensor_msgs::msg::Image>("image/obstacle_visualized", 10);

        // Initialize synchronizer
        m_obstacle_bbox_visualizer_sync = std::make_shared<ObstacleBboxVisualizerSync>(
            ObstacleBboxVisualizerPolicy(10), m_image_sub, m_obstacle_map_sub, m_bbox_sub);
        m_obstacle_bbox_visualizer_sync->registerCallback(
            std::bind(&ObstacleBboxVisualizer::image_obstacle_cb, this, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3));

        this->declare_parameter("color_label_mappings_file", "");
        m_color_label_mappings_file = this->get_parameter("color_label_mappings_file").as_string();
        std::ifstream yamlFile(m_color_label_mappings_file);

        if (yamlFile.is_open()) {
            config_yaml = YAML::Load(yamlFile);
            for (YAML::const_iterator it = config_yaml.begin(); it != config_yaml.end(); ++it) {
                m_label_color_map[it->second.as<int>()] = it->first.as<std::string>();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s",
                         m_color_label_mappings_file.c_str());
        }

        yamlFile.close();
    }

private:
    void image_obstacle_cb(
        const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
        const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg,
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {

        // Get the transform from LiDAR to camera
        if (!m_pc_cam_tf_ok) {
            m_pc_cam_tf = get_tf(in_bbox_msg->header.frame_id, in_map_msg->local_header.frame_id);
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        for (const auto &obstacle : in_map_msg->obstacles) {

            // Transform the lidar point to camera frame
            geometry_msgs::msg::Point lidar_point;
            lidar_point.x = obstacle.local_point.point.x;
            lidar_point.y = obstacle.local_point.point.y;
            lidar_point.z = obstacle.local_point.point.z;
            geometry_msgs::msg::Point camera_point;
            tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, camera_point, m_pc_cam_tf);

            // Find the centroid and display it.
            cv::Point2d pixel_centroid =
                m_is_sim ? m_cam_model.project3dToPixel(
                               cv::Point3d(camera_point.y, camera_point.z, -camera_point.x))
                         : m_cam_model.project3dToPixel(
                               cv::Point3d(camera_point.x, camera_point.y, camera_point.z));
            if (pixel_centroid.x >= 0 && pixel_centroid.x < cv_ptr->image.cols &&
                pixel_centroid.y >= 0 && pixel_centroid.y < cv_ptr->image.rows) {
                cv::Scalar color = get_color_for_label(obstacle.label);

                cv::circle(cv_ptr->image, pixel_centroid, 5, color, -1);
            } else {
                RCLCPP_WARN(this->get_logger(), "Centroid outside image bounds: (%f, %f)",
                            pixel_centroid.x, pixel_centroid.y);
            }

            // Display bbox -> 2d bbox. This is the 2d bbox of the lidar point.
            geometry_msgs::msg::Point camera_bbox_min;
            geometry_msgs::msg::Point camera_bbox_max;
            tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_min, camera_bbox_min,
                                                        m_pc_cam_tf);
            tf2::doTransform<geometry_msgs::msg::Point>(obstacle.bbox_max, camera_bbox_max,
                                                        m_pc_cam_tf);
            cv::Point pt1 =
                m_is_sim ? m_cam_model.project3dToPixel(
                               cv::Point3d(camera_bbox_min.y, camera_bbox_min.z, -camera_bbox_min.x))
                         : m_cam_model.project3dToPixel(
                               cv::Point3d(camera_bbox_min.x, camera_bbox_min.y, camera_bbox_min.z));
            cv::Point pt2 =
                m_is_sim ? m_cam_model.project3dToPixel(
                               cv::Point3d(camera_bbox_max.y, camera_bbox_max.z, -camera_bbox_max.x))
                         : m_cam_model.project3dToPixel(
                               cv::Point3d(camera_bbox_max.x, camera_bbox_max.y, camera_bbox_max.z));
            cv::Scalar color(0, 255, 255);
            cv::rectangle(cv_ptr->image, pt1, pt2, color, 2);

            // Define the position for the label text, slightly above the bounding box
            cv::Point label_position(pt1.x, pt1.y - 5); // Adjust to position above the rectangle

            // Add label text
            std::string label_text =
                std::to_string(obstacle.label); // Replace with whatever string you want
            cv::putText(cv_ptr->image, label_text, label_position, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        color, 1);
        }

        // Draw the bounding boxes
        for (const auto &bbox : in_bbox_msg->boxes) {
            cv::Scalar color = get_color_for_label(bbox.label);
            cv::Point pt1(bbox.min_x, bbox.min_y);
            cv::Point pt2(bbox.max_x, bbox.max_y);
            cv::rectangle(cv_ptr->image, pt1, pt2, color, 2);
        }

        m_image_pub->publish(*cv_ptr->toImageMsg());
    }

    void intrinsics_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) {
        m_cam_model.fromCameraInfo(info_msg);
    }

    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame) {
        geometry_msgs::msg::TransformStamped tf;
        m_pc_cam_tf_ok = false;
        try {
            tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
            m_pc_cam_tf_ok = true;
            RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good in visualizer");
            RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s",
                        in_target_frame.c_str(), in_src_frame.c_str());
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Error in LiDAR to Camera TF in visualizer: %s",
                         ex.what());
        }
        return tf;
    }

    cv::Scalar get_color_for_label(const int &label) {
        std::string color = m_label_color_map[label];

        // In BGR format
        if (color == "orange")
            return cv::Scalar(0, 165, 255);
        if (color == "red")
            return cv::Scalar(0, 0, 255);
        if (color == "green")
            return cv::Scalar(0, 255, 0);
        if (color == "black")
            return cv::Scalar(0, 0, 0);
        if (color == "white")
            return cv::Scalar(255, 255, 255);
        if (color == "yellow")
            return cv::Scalar(0, 255, 255);
        return cv::Scalar(255, 0, 0); // blue
    }

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_image_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::ObstacleMap> m_obstacle_map_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> m_bbox_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_pc_cam_tf_ok;
    std::string m_color_label_mappings_file;
    YAML::Node config_yaml;
    std::map<int, std::string> m_label_color_map;

    // Camera model
    image_geometry::PinholeCameraModel m_cam_model;

    // Check if running in sim
    bool m_is_sim;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        all_seaing_interfaces::msg::ObstacleMap,
        all_seaing_interfaces::msg::LabeledBoundingBox2DArray>
        ObstacleBboxVisualizerPolicy;
    typedef message_filters::Synchronizer<ObstacleBboxVisualizerPolicy> ObstacleBboxVisualizerSync;
    std::shared_ptr<ObstacleBboxVisualizerSync> m_obstacle_bbox_visualizer_sync;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleBboxVisualizer>());
    rclcpp::shutdown();
    return 0;
}
