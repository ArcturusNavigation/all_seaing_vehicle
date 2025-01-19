#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP

#include "all_seaing_perception/obstacle.hpp"

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

#include "all_seaing_interfaces/msg/obstacle_map.hpp"

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector();
    virtual ~ObstacleDetector() = default;

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud);
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>>
    cluster_cloud(builtin_interfaces::msg::Time current_time,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr);
    void
    track_obstacles(builtin_interfaces::msg::Time current_time,
                    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &raw_obstacles);
    void publish_map(std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
                     const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
                     rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub);

    // Member variables
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> m_tracked_obstacles;
    std::string m_global_frame_id;
    int m_obstacle_id;
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    double m_clustering_distance;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_polygon_area_thresh;
    float m_nav_x, m_nav_y, m_nav_heading;

    geometry_msgs::msg::Pose m_nav_pose;

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_raw_map_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_unlabeled_map_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
};

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_DETECTOR_HPP