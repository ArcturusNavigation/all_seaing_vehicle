#ifndef ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP
#define ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <tuple>
#include <chrono>
#include <math.h>
#include <deque>

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
#include "all_seaing_perception/object_tracking_shared.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

class ObjectTrackingMap : public rclcpp::Node{
private:
    void object_track_map_publish(const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &msg);
    void odom_callback();
    void odom_msg_callback(const nav_msgs::msg::Odometry &msg);
    template <typename T>
    T convert_to_global(T point, bool untracked = false);
    template <typename T>
    T convert_to_local(T point, bool untracked = false);

    void update_maps();

    void publish_maps();

    void visualize_predictions();
    
    void publish_slam();

    // Member variables
    std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>>> m_tracked_obstacles;
    std::string m_global_frame_id, m_local_frame_id, m_slam_frame_id;
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header, m_global_untracked_header;
    int m_obstacle_id;
    
    double m_obstacle_drop_thresh, m_range_drop_thresh;
    double m_init_new_cov, m_init_xy_noise, m_init_theta_noise;
    bool m_track_robot, m_imu_predict, m_gps_update;
    double m_normalize_drop_dist;
    double m_odom_refresh_rate;
    bool m_is_sim;
    bool m_direct_tf;
    bool m_normalize_drop_thresh;
    bool m_include_odom_theta, m_include_odom_only_theta;
    std::string m_data_association_algo;
    double m_trace_time;
    bool m_include_unlabeled;
    double m_unlabeled_assoc_threshold;

    double m_nav_x, m_nav_y, m_nav_z, m_nav_heading, m_nav_omega, m_nav_vx, m_nav_vy, m_nav_vz;
    rclcpp::Time m_last_odom_time;
    
    std::deque<std::tuple<float, float, float>> m_trace; // (x,y,time)

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_tracked_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_map_cov_viz_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_slam_pub;
    rclcpp::Subscription<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_object_sub;
    rclcpp::Subscription<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_unlabeled_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    geometry_msgs::msg::TransformStamped m_base_link_map_tf, m_map_base_link_tf;
    rclcpp::TimerBase::SharedPtr odom_timer;

    //SLAM matrices & variables
    float m_range_std, m_bearing_std, m_new_obj_slam_thres;
    float m_gps_xy_noise, m_gps_theta_noise;
    float m_imu_xy_noise, m_imu_theta_noise;
    float m_update_gps_xy_uncertainty, m_update_odom_theta_uncertainty;
    int m_num_obj;
    Eigen::VectorXf m_state;//obstacle map
    Eigen::MatrixXf m_cov;//covariance matrix
    bool m_first_state, m_got_local_frame, m_got_nav, m_got_odom, m_rotate_odom;
    nav_msgs::msg::Odometry m_last_odom_msg;
public:
    ObjectTrackingMap();
    virtual ~ObjectTrackingMap();
};

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP