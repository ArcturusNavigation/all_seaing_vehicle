#ifndef ALL_SEAING_PERCEPTION__OBJECT_TRACKING_SHARED_HPP
#define ALL_SEAING_PERCEPTION__OBJECT_TRACKING_SHARED_HPP


#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <tuple>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/distances.h>
#include "pcl_conversions/pcl_conversions.h"

#include "cv_bridge/cv_bridge.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "all_seaing_interfaces/msg/labeled_object_point_cloud_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"

#include "all_seaing_perception/obstacle.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "all_seaing_perception/Hungarian.h"

namespace all_seaing_perception {
    //custom struct to also keep the points themselves with the obstacle (Obstacle doesn't do that and don't want to mess with it)    
    template<typename PointT>
    struct ObjectCloud{
        int label;
        rclcpp::Time time_seen;
        rclcpp::Time last_dead;
        rclcpp::Duration time_dead = rclcpp::Duration(0,0);
        bool is_dead;
        all_seaing_perception::Obstacle<PointT> obstacle;
        Eigen::Vector2f mean_pred;
        Eigen::Matrix2f cov; 

        ObjectCloud(): obstacle(all_seaing_perception::Obstacle<PointT>(std_msgs::msg::Header(), std_msgs::msg::Header(), all_seaing_interfaces::msg::Obstacle())){
        
        }
        ObjectCloud(rclcpp::Time t, int l, all_seaing_perception::Obstacle<PointT> obs);
    };

    template<typename PointT>
    std::shared_ptr<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::shared_ptr<ObjectCloud<PointT>> orig);

    template<typename PointT>
    std::vector<std::shared_ptr<ObjectCloud<PointT>>> clone(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> orig);

    double mod_2pi(double angle);
    double angle_to_pi_range(double angle);

    template<typename PointT>
    std::tuple<float, float, int> local_to_range_bearing_signature(PointT point, int label);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped get_tf(const std::unique_ptr<tf2_ros::Buffer> &tf_buffer,
        const std::string &in_target_frame, const std::string &in_src_frame);
                                                
    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);
    std::tuple<double, double, double> compose_transforms(std::tuple<double, double, double> t1, std::tuple<double, double, double> t2);
    std::tuple<double, double, double> apply_transform_from_to(double x, double y, double theta, double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);

    // Returns the matchings from detections to map and the sets of indices of chosen tracked and detected obstacles
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> indiv_greedy_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres);
    template<typename PointT>
    std::tuple<std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> linear_sum_assignment_data_association(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, float new_obj_thres, bool sqrt=false);

    // Similar to the greedy_data_association function but returning the computed weight as well
    template<typename PointT>
    std::tuple<float, std::vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> tracked_obstacles,
        typename std::vector<std::shared_ptr<ObjectCloud<PointT>>> detected_obstacles,
        std::vector<std::vector<float>> p, std::vector<std::vector<float>> probs, float new_obj_thres);

} // namespace all_seaing_perception

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_SHARED_HPP