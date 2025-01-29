#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP

#include "all_seaing_perception/obstacle.hpp"

#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "all_seaing_interfaces/msg/obstacle_map.hpp"

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector();
    virtual ~ObstacleDetector() = default;

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud);
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>>
    cluster_cloud(builtin_interfaces::msg::Time current_time,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr);
    void track_obstacles(builtin_interfaces::msg::Time current_time,
                         std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &raw_obstacles);
    void publish_map(std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
                     const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
                     rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub);
    void set_odom();

    // Member variables
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> m_tracked_obstacles;
    std::string m_global_frame_id;
    std::string m_robot_frame_id;
    int m_obstacle_id;
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    double m_clustering_distance;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_polygon_area_thresh;
    float m_nav_x, m_nav_y, m_nav_heading;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_raw_map_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_unlabeled_map_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
};

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP
