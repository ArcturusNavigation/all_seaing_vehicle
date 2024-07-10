#ifndef ALL_SEAING_PERCEPTION_OBSTACLE_DETECTOR_HPP
#define ALL_SEAING_PERCEPTION_OBSTACLE_DETECTOR_HPP

#include "all_seaing_perception/obstacle.hpp"

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "protobuf_client_interfaces/msg/gateway.hpp"

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector();
    virtual ~ObstacleDetector() = default;

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud);
    void odom_callback(const nav_msgs::msg::Odometry &msg);
    void segment_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                       all_seaing_interfaces::msg::ObstacleMap &out_map);
    std::vector<std::shared_ptr<Obstacle>>
    cluster_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr);
    void match_obstacles(std::vector<std::shared_ptr<Obstacle>> &raw_obstacles,
                         std::vector<std::shared_ptr<Obstacle>> &tracked_obstacles);
    void markers(const all_seaing_interfaces::msg::ObstacleMap &in_map);
    void send_to_gateway(const all_seaing_interfaces::msg::ObstacleMap &in_map);

    // Member variables
    std::vector<std::shared_ptr<Obstacle>> m_tracked_obstacles;
    std_msgs::msg::Header m_lidar_header;
    builtin_interfaces::msg::Time m_current_time;
    int m_obstacle_id;
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    double m_clustering_distance;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_polygon_area_thresh;
    bool m_viz;
    float m_nav_x, m_nav_y, m_nav_heading;

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_obstacle_cloud_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_raw_map_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr
        m_unlabeled_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        m_marker_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        m_text_marker_array_pub;
    rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr
        m_gateway_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
};

#endif // ALL_SEAING_PERCEPTION_OBSTACLE_DETECTOR_HPP