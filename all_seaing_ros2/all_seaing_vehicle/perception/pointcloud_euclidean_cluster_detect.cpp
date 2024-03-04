#include "all_seaing_vehicle/pointcloud_euclidean_cluster_detect.hpp"

ClusterNode::ClusterNode() : Node("pointcloud_euclidean_cluster")
{
    // Initialize member variables to default values
    initMemberVariables();

    // Initialize publishers and subscribers
    initPublishersAndSubscribers();
}

// Implement other member functions of ClusterNode
void ClusterNode::initMemberVariables()
{
    _using_cloud = false;
    _filter_camera_view = false;
    _camera_hfov = 80;
    _camera_theta = 0.0;
    _filter_cloud = false;
    _cull_min = 0.0;
    _cull_max = 0.0;
    _downsampled_cloud = false;
    _leaf_size = 0.0;
    _cluster_size_min = 20;
    _cluster_size_max = 100000;
    _clustering_distance = 0.75;
    _use_multiple_thres = false;
    _cluster_seg_thresh = 1.0;
    _drop_cluster_count = 5;
    _drop_cluster_thresh = 1.0;
    _polygon_area_thresh = 100000.0;
    _viz = true;
}

void ClusterNode::initPublishersAndSubscribers()
{
    _pub_cluster_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud", 10);
    _centroid_pub = this->create_publisher<all_seaing_interfaces::msg::Centroids>("/cluster_centroids", 10);
    _pub_clusters_message = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/detection/raw_cloud_clusters", 10);
    _pub_matched_clusters_msg = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/detection/matched_cloud_clusters", 10);
    _marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/chull_markers", 10);
    _text_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/text_markers", 10);

    _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/wamv/sensors/lidars/filtered/points", 100, std::bind(&ClusterNode::pc_callback, this, std::placeholders::_1));
}

// Define other member functions like pc_callback, filterCloud, downsampleCloud, etc.
// ...
/**
 * @brief Filter points around the vessel to remove self reflections
 * @param in_cloud_ptr  Input point cloud to filter
 * @param out_cloud_ptr Output point cloud to store filtered cloud
 * @param min_distance  Minimum distance allowed to keep a point
 * @param max_distance  Maximum distance allowed to keep a point
 */
void ClusterNode::filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                              double min_distance,
                              double max_distance)
{
    out_cloud_ptr->points.clear();
    for (const auto &point : in_cloud_ptr->points)
    {
        float origin_distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
        if (origin_distance > min_distance && origin_distance < max_distance)
        {
            out_cloud_ptr->points.push_back(point);
        }
    }
}

void ClusterNode::downsampledCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                                  float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in_cloud_ptr);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*out_cloud_ptr);
}

void ClusterNode::filterCameraView(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
                                   const int fov, const double theta)
{
    float half_fov = static_cast<float>(fov * M_PI / 180) / 2.0f;
    float camera_theta = static_cast<float>(theta * M_PI / 180);

    out_cloud_ptr->points.clear();
    for (const auto &point : in_cloud_ptr->points)
    {
        float current_theta = std::atan2(point.y, point.x);
        if (current_theta > camera_theta - half_fov && current_theta < camera_theta + half_fov)
        {
            out_cloud_ptr->points.push_back(point);
        }
    }
}

void ClusterNode::ClusterNode::segmentCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
                                            all_seaing_interfaces::msg::Centroids &in_out_centroids,
                                            all_seaing_interfaces::msg::CloudClusterArray &in_out_clusters)
{
    // Cluster the pointcloud by the distance of the points using different thresholds
    // This helps with clustering at larger distances. See adaptive clustering for more info.
    // Thresholds are based off empirical evidence, further study required for larger distances

    std::vector<std::shared_ptr<Cluster>> all_clusters;
    if (!_use_multiple_thres)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*in_cloud_ptr));
        all_clusters = clusterCloud(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);
    }

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

        pcl::PointXYZ center_point = all_clusters[i]->GetCentroidPoint();
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
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud_ptr,
    all_seaing_interfaces::msg::Centroids &in_out_centroids,
    double in_max_cluster_distance)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    // Create 2d pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);

    // Flatten cloud
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    if (!cloud_2d->points.empty())
        tree->setInputCloud(cloud_2d);

    // Extract clusters from point cloud and save indices in cluster_indices
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
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

// Publishing functions
// Function to publish point cloud
void ClusterNode::publishCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud_to_publish_ptr)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _sensor_header; // Assuming _sensor_header is properly set elsewhere in your class

    _pub_cluster_cloud->publish(cloud_msg);
}
// Function to publish centroids
void ClusterNode::publishCentroids(const all_seaing_interfaces::msg::Centroids &in_centroids)
{
    // Use the publisher member to publish the centroids message
    _centroid_pub->publish(in_centroids);
}
// Function to publish cloud clusters
void ClusterNode::publishCloudClusters(const all_seaing_interfaces::msg::CloudClusterArray &in_clusters)
{
    // Directly use the publisher to publish the message
    _pub_clusters_message->publish(in_clusters);
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
        text_marker.scale.z = 10.0; // Text scale in RVIZ
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
    if (!_using_cloud)
    {
        _using_cloud = true;
        _current_time = in_cloud->header.stamp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

        all_seaing_interfaces::msg::Centroids centroids;              // Current segment centroids
        all_seaing_interfaces::msg::CloudClusterArray cloud_clusters; // Current segments clusters

        fromROSMsg(*in_cloud, *current_cloud_ptr);
        _sensor_header = in_cloud->header;

        if (_filter_cloud)
            filterCloud(current_cloud_ptr, filtered_cloud_ptr, _cull_min, _cull_max);
        else
            filtered_cloud_ptr = current_cloud_ptr;

        if (_downsampled_cloud)
            downsampledCloud(filtered_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
        else
            downsampled_cloud_ptr = filtered_cloud_ptr;

        if (_filter_camera_view)
            filterCameraView(downsampled_cloud_ptr, camera_cloud_ptr, _camera_hfov, _camera_theta);
        else
            camera_cloud_ptr = downsampled_cloud_ptr;

        segmentCloud(camera_cloud_ptr, clustered_cloud_ptr, centroids, cloud_clusters);

        // Cluster_cloud publisher (raw cloud publisher)
        publishCloud(clustered_cloud_ptr);

        centroids.header = _sensor_header;
        publishCentroids(centroids);

        // Cloud clusters message publisher (matched clusters)
        cloud_clusters.header = _sensor_header;
        publishCloudClusters(cloud_clusters);

        // Visualization
        if (_viz)
            markers(cloud_clusters);

        _using_cloud = false;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
