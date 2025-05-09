#ifndef ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_EUCLIDEAN_HPP
#define ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_EUCLIDEAN_HPP

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

    ObjectCloud(rclcpp::Time t, int l, pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc, pcl::PointCloud<pcl::PointXYZHSV>::Ptr glob);

    void update_loc_pcloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc);
};

class ObjectTrackingMapEuclidean : public rclcpp::Node{
private:
    void object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg);
    void odom_callback();
    void publish_map(std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
                     const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
                     rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub, std::vector<int> labels);
    template <typename T>
    T convert_to_global(T point);
    template <typename T>
    T convert_to_local(T point);

    // Get intrinsic camera model information needed for projection
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame);
    
    // Member variables
    std::vector<std::shared_ptr<ObjectCloud>> m_tracked_obstacles;
    std::string m_global_frame_id, m_local_frame_id;
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_obstacle_id;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_odom_refresh_rate;

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_untracked_map_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_tracked_map_pub;
    rclcpp::Subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>::SharedPtr m_object_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_lidar_map_tf, m_map_lidar_tf;
    rclcpp::TimerBase::SharedPtr odom_timer;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    bool m_is_sim;
    bool m_check_fov;
    bool m_got_local_frame;
public:
    ObjectTrackingMapEuclidean();
    virtual ~ObjectTrackingMapEuclidean();
};

#endif // ALL_SEAING_PERCEPTION__OBJECT_TRACKING_MAP_EUCLIDEAN_HPP