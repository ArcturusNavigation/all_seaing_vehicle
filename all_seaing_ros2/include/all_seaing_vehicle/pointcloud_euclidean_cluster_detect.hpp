#ifndef CLUSTERNODE_HPP
#define CLUSTERNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <unordered_set>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"

#include "protobuf_client_interfaces/msg/gateway.hpp"
#include "all_seaing_vehicle/cluster.hpp"

class ClusterNode : public rclcpp::Node
{
public:
    ClusterNode();
    virtual ~ClusterNode() = default;

private:
    void pcCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud);
    void odomCallback(const nav_msgs::msg::Odometry &msg);
    void segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                      all_seaing_interfaces::msg::CloudClusterArray &in_out_clusters);
    std::vector<std::shared_ptr<Cluster>> clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr);
    void matchClusters(std::vector<std::shared_ptr<Cluster>> &in_clusters,
                       std::vector<std::shared_ptr<Cluster>> &in_out_clusters);
    void markers(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array);
    void sendToGateway(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array);
    geometry_msgs::msg::Point convertToGlobal(const geometry_msgs::msg::Point &point);
    geometry_msgs::msg::Point convertToGlobal(const geometry_msgs::msg::Point32 &point);

    // Member variables
    std::vector<std::shared_ptr<Cluster>> _tracked_clusters;
    std_msgs::msg::Header _sensor_header;
    builtin_interfaces::msg::Time _current_time;
    int _cluster_size_min;
    int _cluster_size_max;
    int _cluster_id;
    double _clustering_distance;
    double _cluster_seg_thresh;
    double _drop_cluster_thresh;
    double _polygon_area_thresh;
    bool _viz;
    float nav_x_, nav_y_, nav_heading_;

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cluster_cloud;
    rclcpp::Publisher<all_seaing_interfaces::msg::CloudClusterArray>::SharedPtr _pub_clusters_message;
    rclcpp::Publisher<all_seaing_interfaces::msg::CloudClusterArray>::SharedPtr _pub_matched_clusters_msg;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _text_marker_array_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_publisher;
    rclcpp::Publisher<protobuf_client_interfaces::msg::Gateway>::SharedPtr _gateway_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
};

#endif // CLUSTERNODE_HPP
