#ifndef ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP
#define ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include <vector>
#include <tuple>
#include <chrono>

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

//custom struct to also keep the points themselves with the obstacle (Obstacle doesn't do that and don't want to mess with it)
struct ObjectCloud{
    int id;
    int label;
    rclcpp::Time time_seen;
    rclcpp::Time last_dead;
    rclcpp::Duration time_dead = rclcpp::Duration(0,0);
    bool is_dead;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_pcloud_ptr;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_pcloud_ptr;
    pcl::PointXYZ local_centroid;
    pcl::PointXYZ global_centroid;
    Eigen::Vector2f mean_pred;
    Eigen::Matrix2f cov; 

    ObjectCloud(rclcpp::Time t, int l, pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc, pcl::PointCloud<pcl::PointXYZHSV>::Ptr glob);

    void update_loc_pcloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc);
};

std::shared_ptr<ObjectCloud> clone(std::shared_ptr<ObjectCloud> orig);

class ObjectTrackingMap : public rclcpp::Node{
private:
    void object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg);
    void odom_callback();
    void odom_msg_callback(const nav_msgs::msg::Odometry &msg);
    void publish_map(std_msgs::msg::Header local_header, std_msgs::msg::Header global_header, std::string ns, bool is_labeled,
                     const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
                     rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub, std::vector<int> labels);
    template <typename T>
    T convert_to_global(T point);
    template <typename T>
    T convert_to_local(T point);

    void visualize_predictions();

    typedef std::tuple<float, float, int> det_rbs;
    template <typename T>
    det_rbs local_to_range_bearing_signature(T point, int label);

    // Get intrinsic camera model information needed for projection
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped ObjectTrackingMap::get_tf(const std::unique_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string &in_target_frame, const std::string &in_src_frame);
                                                
    std::tuple<double, double, double> compute_transform_from_to(double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);
    std::tuple<double, double, double> compose_transforms(std::tuple<double, double, double> t1, std::tuple<double, double, double> t2);
    std::tuple<double, double, double> apply_transform_from_to(double x, double y, double theta, double from_x, double from_y, double from_theta, double to_x, double to_y, double to_theta);
    
    void publish_slam();

    // Returns the matchings from detections to map and the sets of indices of chosen tracked and detected obstacles
    std::tuple<vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        vector<vector<float>> p, float new_obj_thres);
    
    // Similar to the greedy_data_association function but returning the computed weight as well
    std::tuple<float, vector<int>, std::unordered_set<int>, std::unordered_set<int>> greedy_data_association_probs(std::vector<std::shared_ptr<ObjectCloud>> tracked_obstacles,
        std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles,
        vector<vector<float>> p, vector<vector<float>> probs, float new_obj_thres);

    // Member variables
    std::vector<std::shared_ptr<ObjectCloud>> m_tracked_obstacles;
    std::string m_global_frame_id, m_local_frame_id, m_slam_frame_id;
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_obstacle_id;
    double m_obstacle_drop_thresh;
    double m_init_new_cov;
    bool m_track_robot, m_imu_predict, m_gps_update;
    double m_normalize_drop_dist;
    double m_odom_refresh_rate;

    double m_nav_x, m_nav_y, m_nav_z, m_nav_heading, m_nav_omega, m_nav_vx, m_nav_vy, m_nav_vz;
    rclcpp::Time m_last_odom_time;

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_untracked_map_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_tracked_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_map_cov_viz_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_slam_pub;
    rclcpp::Subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>::SharedPtr m_object_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    geometry_msgs::msg::TransformStamped m_lidar_map_tf, m_map_lidar_tf;
    rclcpp::TimerBase::SharedPtr odom_timer;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    //SLAM matrices & variables
    float m_range_std, m_bearing_std, m_new_obj_slam_thres;
    float m_gps_xy_noise, m_gps_theta_noise;
    float m_imu_xy_noise, m_imu_theta_noise;
    float m_update_gps_xy_uncertainty, m_update_odom_theta_uncertainty;
    int m_num_obj;
    Eigen::VectorXf m_state;//obstacle map
    Eigen::MatrixXf m_cov;//covariance matrix
    bool m_first_state, m_got_local_frame, m_got_nav, m_got_odom;
    nav_msgs::msg::Odometry m_last_odom_msg;

    bool m_is_sim;
    bool m_check_fov;
    bool m_direct_tf;
    bool m_normalize_drop_thresh;
    bool m_include_odom_theta;
public:
    ObjectTrackingMap();
    virtual ~ObjectTrackingMap();
};

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_HPP