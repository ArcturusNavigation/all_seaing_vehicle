#ifndef CLUSTERNODE_HPP
#define CLUSTERNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <vector>
#include <cmath>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/header.hpp"

#include "all_seaing_vehicle/cluster.hpp"

class ClusterNode : public rclcpp::Node
{
public:
    ClusterNode();
    virtual ~ClusterNode() = default;

private:
    void initMemberVariables();
    void initPublishersAndSubscribers();
    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud);
    void filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                     double min_distance,
                     double max_distance);
    void downsampledCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                          float leaf_size = 0.2);
    void filterCameraView(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                          const int fov, const double theta);
    void segmentCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                      all_seaing_interfaces::msg::Centroids &in_out_centroids,
                      all_seaing_interfaces::msg::CloudClusterArray &in_out_clusters);
    std::vector<std::shared_ptr<Cluster>> clusterCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
        all_seaing_interfaces::msg::Centroids &in_out_centroids,
        double in_max_cluster_distance = 0.5);
    void matchClusters(std::vector<std::shared_ptr<Cluster>> &in_clusters,
                       std::vector<std::shared_ptr<Cluster>> &in_out_clusters);
    void publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr);
    void publishCentroids(const all_seaing_interfaces::msg::Centroids &in_centroids);
    void publishCloudClusters(const all_seaing_interfaces::msg::CloudClusterArray &in_clusters);
    void markers(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array);

    // Member variables
    std::vector<std::shared_ptr<Cluster>> _tracked_clusters;
    std_msgs::msg::Header _sensor_header;
    builtin_interfaces::msg::Time _current_time;
    int _cluster_size_min;
    int _cluster_size_max;
    int cluster_id;
    bool _using_cloud;
    bool _filter_camera_view;
    int _camera_hfov;
    double _camera_theta;
    bool _filter_cloud;
    double _cull_min;
    double _cull_max;
    bool _downsampled_cloud;
    double _leaf_size;
    double _clustering_distance;
    bool _use_multiple_thres;
    std::string _output_frame;
    double _cluster_seg_thresh;
    int _drop_cluster_count;
    double _drop_cluster_thresh;
    double _polygon_area_thresh;
    bool _viz;

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cluster_cloud;
    rclcpp::Publisher<all_seaing_interfaces::msg::Centroids>::SharedPtr _centroid_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::CloudClusterArray>::SharedPtr _pub_clusters_message;
    rclcpp::Publisher<all_seaing_interfaces::msg::CloudClusterArray>::SharedPtr _pub_matched_clusters_msg;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_array_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _text_marker_array_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _cloud_sub;
};

#endif // CLUSTERNODE_HPP
