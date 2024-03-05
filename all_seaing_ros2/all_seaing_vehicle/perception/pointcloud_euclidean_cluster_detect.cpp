#include "all_seaing_vehicle/pointcloud_euclidean_cluster_detect.hpp"

ClusterNode::ClusterNode() : Node("pointcloud_euclidean_cluster")
{
    // Initialize parameters
    this->declare_parameter<int>("cluster_size_min", 20);
    this->declare_parameter<int>("cluster_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    this->declare_parameter<double>("cluster_seg_thresh", 1.0);
    this->declare_parameter<int>("drop_cluster_count", 5);
    this->declare_parameter<double>("drop_cluster_thresh", 1.0);
    this->declare_parameter<double>("polygon_area_thresh", 100000.0);
    this->declare_parameter<bool>("viz", true);

    // Initialize member variables from parameters
    _cluster_size_min = this->get_parameter("cluster_size_min").as_int();
    _cluster_size_max = this->get_parameter("cluster_size_max").as_int();
    _clustering_distance = this->get_parameter("clustering_distance").as_double();
    _cluster_seg_thresh = this->get_parameter("cluster_seg_thresh").as_double();
    _drop_cluster_count = this->get_parameter("drop_cluster_count").as_int();
    _drop_cluster_thresh = this->get_parameter("drop_cluster_thresh").as_double();
    _polygon_area_thresh = this->get_parameter("polygon_area_thresh").as_double();
    _viz = this->get_parameter("viz").as_bool();
    
    // Initialize publishers and subscribers
    _pub_cluster_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud", 10);
    _centroid_pub = this->create_publisher<all_seaing_interfaces::msg::Centroids>("/cluster_centroids", 10);
    _pub_clusters_message = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/detection/raw_cloud_clusters", 10);
    _pub_matched_clusters_msg = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/detection/matched_cloud_clusters", 10);
    _marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/chull_markers", 10);
    _text_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/text_markers", 10);
    _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wamv/sensors/lidars/filtered/points", 100, std::bind(&ClusterNode::pc_callback, this, std::placeholders::_1));
}

void ClusterNode::segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                               all_seaing_interfaces::msg::Centroids &in_out_centroids,
                               all_seaing_interfaces::msg::CloudClusterArray &in_out_clusters)
{
    // Cluster the pointcloud by the distance of the points
    std::vector<std::shared_ptr<Cluster>> all_clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*in_cloud_ptr));
    all_clusters = clusterCloud(cloud_ptr, _clustering_distance);

    // Push raw clusters (all_clusters) to CloudClusterArray and publish
    all_seaing_interfaces::msg::CloudClusterArray raw_cloud_clusters;
    for (unsigned int i = 0; i < all_clusters.size(); i++)
    {
        if (all_clusters[i]->IsValid())
        {
            all_seaing_interfaces::msg::CloudCluster raw_cloud_cluster;
            all_clusters[i]->ToROSMessage(_sensor_header, raw_cloud_cluster);
            raw_cloud_clusters.clusters.push_back(raw_cloud_cluster);
        }
    }

    // Use the shared pointer publisher to publish the clusters
    _pub_clusters_message->publish(raw_cloud_clusters);

    // Current sensor clusters (all_clusters), tracked_clusters
    matchClusters(all_clusters, _tracked_clusters);

    // Final pointcloud to be published
    for (unsigned int i = 0; i < all_clusters.size(); i++)
    {
        *out_cloud_ptr += *all_clusters[i]->GetCloud();

        pcl::PointXYZI center_point = all_clusters[i]->GetCentroidPoint();
        geometry_msgs::msg::Point centroid;
        centroid.x = center_point.x;
        centroid.y = center_point.y;
        centroid.z = center_point.z;

        // Push matched clusters to CloudClusterArray
        if (all_clusters[i]->IsValid())
        {
            in_out_centroids.points.push_back(centroid);

            all_seaing_interfaces::msg::CloudCluster cloud_cluster;
            all_clusters[i]->ToROSMessage(_sensor_header, cloud_cluster);
            in_out_clusters.clusters.push_back(cloud_cluster);
        }
    }
}

std::vector<std::shared_ptr<Cluster>> ClusterNode::clusterCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
    double in_max_cluster_distance)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());

    // Create 2d pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);

    // Flatten cloud
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    if (!cloud_2d->points.empty())
        tree->setInputCloud(cloud_2d);

    // Extract clusters from point cloud and save indices in cluster_indices
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(in_max_cluster_distance);
    ec.setMinClusterSize(_cluster_size_min);
    ec.setMaxClusterSize(_cluster_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(cluster_indices);

    // Cluster Points
    std::vector<std::shared_ptr<Cluster>> clusters;

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        std::shared_ptr<Cluster> cluster(new Cluster());
        cluster->SetCloud(in_cloud_ptr, it->indices, _sensor_header, cluster_id++, "", _current_time);
        clusters.push_back(cluster);
    }

    return clusters;
}
void ClusterNode::matchClusters(std::vector<std::shared_ptr<Cluster>> &in_clusters,
                                std::vector<std::shared_ptr<Cluster>> &in_out_clusters)
{
    std::vector<std::shared_ptr<Cluster>> track_clusters;
    for (int i = 0; i < in_out_clusters.size(); i++)
    {
        rclcpp::Duration current_time(_current_time.sec, _current_time.nanosec);
        rclcpp::Duration cluster_last_seen(in_out_clusters[i]->GetLastSeen().sec, in_out_clusters[i]->GetLastSeen().nanosec);
        rclcpp::Duration last_seen = current_time - cluster_last_seen;

        if (last_seen.seconds() >= _drop_cluster_thresh)
        {
            in_out_clusters.erase(in_out_clusters.begin() + (i > 0 ? i - 1 : 0));
        }
    }

    for (auto &cluster : in_clusters)
    {
        if (cluster->GetPolygonArea() > this->_polygon_area_thresh)
        {
            continue;
        }

        float dist = this->_cluster_seg_thresh;
        int best_match = -1;

        for (int j = 0; j < in_out_clusters.size(); j++)
        {
            float this_dist = euclideanDistance(in_out_clusters[j]->GetCentroidPoint(), cluster->GetCentroidPoint());
            if (this_dist < dist)
            {
                best_match = j;
                dist = this_dist;
            }
        }

        if (best_match >= 0)
        {
            cluster->SetID(in_out_clusters[best_match]->GetID());
        }

        track_clusters.push_back(cluster);
    }

    if (in_out_clusters.empty())
    {
        in_out_clusters = std::move(in_clusters);
    }
    else
    {
        in_out_clusters.insert(in_out_clusters.end(), track_clusters.begin(), track_clusters.end());
    }
}

void ClusterNode::markers(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array)
{
    visualization_msgs::msg::MarkerArray markers_array;
    visualization_msgs::msg::MarkerArray text_markers_array;

    for (const auto &cluster : in_cluster_array.clusters)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = cluster.header.frame_id;
        marker.ns = "clustering";
        marker.id = cluster.id;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(1.3);

        visualization_msgs::msg::Marker text_marker;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.header = cluster.header;
        text_marker.ns = "text_clustering";
        text_marker.id = cluster.id;
        text_marker.scale.z = 1.0; // Text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        text_marker.text = std::to_string(cluster.id);

        text_marker.pose.position.x = cluster.max_point.point.x + 3;
        text_marker.pose.position.y = cluster.centroid_point.point.y;
        text_marker.pose.position.z = cluster.centroid_point.point.z;
        text_marker.pose.orientation.x = 0.0;
        text_marker.pose.orientation.y = 0.0;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_markers_array.markers.push_back(text_marker);

        for (const auto &p : cluster.convex_hull.polygon.points)
        {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            marker.points.push_back(point);
        }

        // Close the loop for LINE_STRIP
        if (!cluster.convex_hull.polygon.points.empty())
        {
            geometry_msgs::msg::Point first_point;
            first_point.x = cluster.convex_hull.polygon.points.front().x;
            first_point.y = cluster.convex_hull.polygon.points.front().y;
            first_point.z = cluster.convex_hull.polygon.points.front().z;
            marker.points.push_back(first_point);
        }

        markers_array.markers.push_back(marker);
    }

    _marker_array_pub->publish(markers_array);
    _text_marker_array_pub->publish(text_markers_array);
}

void ClusterNode::pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud)
{
    _current_time = in_cloud->header.stamp;
    _sensor_header = in_cloud->header;

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    all_seaing_interfaces::msg::Centroids centroids;
    all_seaing_interfaces::msg::CloudClusterArray cloud_clusters;

    // Convert ROS2 PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    fromROSMsg(*in_cloud, *current_cloud_ptr);

    // Segment cloud 
    segmentCloud(current_cloud_ptr, clustered_cloud_ptr, centroids, cloud_clusters);

    // Publish clustered clouds (raw cloud publisher)
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*clustered_cloud_ptr, cloud_msg);
    cloud_msg.header = _sensor_header;
    _pub_cluster_cloud->publish(cloud_msg);

    // Publish centroids
    centroids.header = _sensor_header;
    _centroid_pub->publish(centroids);

    // Publish matched cloud clusters
    cloud_clusters.header = _sensor_header;
    _pub_clusters_message->publish(cloud_clusters);

    // Publish visualization markers
    if (_viz) markers(cloud_clusters);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
