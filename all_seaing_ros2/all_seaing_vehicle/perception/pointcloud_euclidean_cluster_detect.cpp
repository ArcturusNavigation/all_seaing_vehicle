#include "all_seaing_vehicle/pointcloud_euclidean_cluster_detect.hpp"

ClusterNode::ClusterNode() : Node("pointcloud_euclidean_cluster")
{
    // Initialize parameters
    this->declare_parameter<int>("cluster_size_min", 20);
    this->declare_parameter<int>("cluster_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    this->declare_parameter<double>("cluster_seg_thresh", 1.0);
    this->declare_parameter<double>("drop_cluster_thresh", 1.0);
    this->declare_parameter<double>("polygon_area_thresh", 100000.0);
    this->declare_parameter<bool>("viz", true);

    // Initialize member variables from parameters
    _cluster_size_min = this->get_parameter("cluster_size_min").as_int();
    _cluster_size_max = this->get_parameter("cluster_size_max").as_int();
    _clustering_distance = this->get_parameter("clustering_distance").as_double();
    _cluster_seg_thresh = this->get_parameter("cluster_seg_thresh").as_double();
    _drop_cluster_thresh = this->get_parameter("drop_cluster_thresh").as_double();
    _polygon_area_thresh = this->get_parameter("polygon_area_thresh").as_double();
    _viz = this->get_parameter("viz").as_bool();

    // Initialize navigation variables to 0
    nav_x_ = 0;
    nav_y_ = 0;
    nav_heading_ = 0;
    
    // Initialize publishers and subscribers
    _pub_cluster_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cluster_cloud", 10);
    _pub_clusters_message = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/raw_cloud_clusters", 10);
    _pub_matched_clusters_msg = this->create_publisher<all_seaing_interfaces::msg::CloudClusterArray>("/matched_cloud_clusters", 10);
    _marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/chull_markers", 10);
    _text_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/text_markers", 10);
    _gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
    _cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/in_cloud", 10, std::bind(&ClusterNode::pcCallback, this, std::placeholders::_1));
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10, std::bind(&ClusterNode::odomCallback, this, std::placeholders::_1));
}

void ClusterNode::segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                               all_seaing_interfaces::msg::CloudClusterArray &in_out_clusters)
{
    // Cluster the pointcloud by the distance of the points
    std::vector<std::shared_ptr<Cluster>> raw_clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(*in_cloud_ptr));
    raw_clusters = clusterCloud(cloud_ptr);

    // Match raw clusters and tracked clusters
    matchClusters(raw_clusters, _tracked_clusters);

    // Push raw clusters to CloudClusterArray and publish
    all_seaing_interfaces::msg::CloudClusterArray raw_cloud_clusters;
    for (unsigned int i = 0; i < raw_clusters.size(); i++)
    {
        all_seaing_interfaces::msg::CloudCluster raw_cloud_cluster;
        raw_clusters[i]->ToROSMessage(_sensor_header, raw_cloud_cluster);
        raw_cloud_clusters.clusters.push_back(raw_cloud_cluster);
    }
    _pub_clusters_message->publish(raw_cloud_clusters);

    // Final pointcloud to be published
    for (unsigned int i = 0; i < _tracked_clusters.size(); i++)
    {
        *out_cloud_ptr += *_tracked_clusters[i]->GetCloud();

        // Push matched clusters to CloudClusterArray
        all_seaing_interfaces::msg::CloudCluster cloud_cluster;
        _tracked_clusters[i]->ToROSMessage(_sensor_header, cloud_cluster);
        in_out_clusters.clusters.push_back(cloud_cluster);
    }
}

std::vector<std::shared_ptr<Cluster>> ClusterNode::clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr)
{
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());

    // Create 2d pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);

    // Flatten cloud
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    // Extract clusters from point cloud and save indices in cluster_indices
    if (!cloud_2d->points.empty()) tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(_clustering_distance);
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
        cluster->SetCloud(in_cloud_ptr, it->indices, _sensor_header, _cluster_id++, _current_time);
        clusters.push_back(cluster);
    }

    return clusters;
}

void ClusterNode::matchClusters(std::vector<std::shared_ptr<Cluster>> &raw_clusters,
                                std::vector<std::shared_ptr<Cluster>> &tracked_clusters)
{
    // Remove tracked clusters if haven't seen for drop cluster threshold
    for (auto it = tracked_clusters.begin(); it != tracked_clusters.end(); it++)
    {
        rclcpp::Duration current_time(_current_time.sec, _current_time.nanosec);
        rclcpp::Duration cluster_last_seen((*it)->GetLastSeen().sec, (*it)->GetLastSeen().nanosec);
        rclcpp::Duration last_seen = current_time - cluster_last_seen;

        if (last_seen.seconds() >= _drop_cluster_thresh)
        {
            it = tracked_clusters.erase(it);
            it--;
        }
            
    }

    std::vector<std::shared_ptr<Cluster>> new_clusters;
    std::unordered_set<int> chosen_indices;
    for (auto it = raw_clusters.begin(); it != raw_clusters.end(); it++)
    {
        auto& raw_cluster = *it;

        // Remove cluster if larger than polygon area threshold
        if (raw_cluster->GetPolygonArea() > _polygon_area_thresh)
        {
            it = raw_clusters.erase(it);
            it--;
            continue;
        }

        // Match clusters based on closest average point
        float best_dist = _cluster_seg_thresh;
        int best_match = -1;
        for (unsigned long j = 0; j < tracked_clusters.size(); j++)
        {
            // Skip indices already chosen
            if (chosen_indices.find(j) != chosen_indices.end())
                continue;

            float curr_dist = euclideanDistance(tracked_clusters[j]->GetAveragePoint(), raw_cluster->GetAveragePoint());
            if (curr_dist < best_dist)
            {
                best_match = j;
                best_dist = curr_dist;
            }
        }

        if (best_match >= 0)
        {
            // If there is best match, then set ID, move to tracked clusters, and remember chosen index
            raw_cluster->SetID(tracked_clusters[best_match]->GetID());
            tracked_clusters[best_match] = raw_cluster;
            chosen_indices.insert(best_match);
        }
        else
        {
            // If there are no matches, then add to new clusters
            new_clusters.push_back(raw_cluster);
        }
    }

    if (tracked_clusters.empty())
        tracked_clusters = std::move(raw_clusters);
    else
        tracked_clusters.insert(tracked_clusters.end(), new_clusters.begin(), new_clusters.end());
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
        marker.scale.x = 0.05;
        marker.lifetime = rclcpp::Duration::from_seconds(1.3);

        visualization_msgs::msg::Marker text_marker;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.header = cluster.header;
        text_marker.ns = "text_clustering";
        text_marker.id = cluster.id;
        text_marker.scale.z = 0.7; // Text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        text_marker.text = std::to_string(cluster.id);

        text_marker.pose.position.x = cluster.max_point.x;
        text_marker.pose.position.y = cluster.avg_point.y;
        text_marker.pose.position.z = cluster.avg_point.z + 1.0;
        text_markers_array.markers.push_back(text_marker);

        for (const auto &p : cluster.convex_hull.points)
        {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            marker.points.push_back(point);
        }

        // Close the loop for LINE_STRIP
        if (!cluster.convex_hull.points.empty())
        {
            geometry_msgs::msg::Point first_point;
            first_point.x = cluster.convex_hull.points.front().x;
            first_point.y = cluster.convex_hull.points.front().y;
            first_point.z = cluster.convex_hull.points.front().z;
            marker.points.push_back(first_point);
        }

        markers_array.markers.push_back(marker);
    }

    _marker_array_pub->publish(markers_array);
    _text_marker_array_pub->publish(text_markers_array);
}

geometry_msgs::msg::Point ClusterNode::convertToGlobal(const geometry_msgs::msg::Point &point)
{
    geometry_msgs::msg::Point new_point;
    double magnitude = std::hypot(point.x, point.y);
    double point_angle = std::atan2(point.y, point.x);
    new_point.x = nav_x_ + std::cos(nav_heading_ + point_angle) * magnitude;
    new_point.y = nav_y_ + std::sin(nav_heading_ + point_angle) * magnitude;
    new_point.z = point.z;
    return new_point;
}

geometry_msgs::msg::Point ClusterNode::convertToGlobal(const geometry_msgs::msg::Point32 &point)
{
    geometry_msgs::msg::Point new_point;
    new_point.x = point.x;
    new_point.y = point.y;
    new_point.z = point.z;
    return convertToGlobal(new_point);
}

void ClusterNode::sendToGateway(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array)
{
    for (const auto &cluster : in_cluster_array.clusters)
    {
        for (const auto &p : cluster.convex_hull.points) {
            auto gateway_msg = protobuf_client_interfaces::msg::Gateway();
            gateway_msg.gateway_key = "TRACKED_FEATURE";
            geometry_msgs::msg::Point p_glob = convertToGlobal(p);
            gateway_msg.gateway_string = 
                "x=" + std::to_string(p_glob.x) + ",y=" + std::to_string(p_glob.y) + ",label=" + std::to_string(cluster.id);
            _gateway_pub->publish(gateway_msg);
        }
    }
}

void ClusterNode::pcCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud)
{
    _current_time = in_cloud->header.stamp;
    _sensor_header = in_cloud->header;

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    all_seaing_interfaces::msg::CloudClusterArray cloud_clusters;

    // Convert ROS2 PointCloud2 to pcl pointcloud
    fromROSMsg(*in_cloud, *current_cloud_ptr);

    // Segment cloud 
    segmentCloud(current_cloud_ptr, clustered_cloud_ptr, cloud_clusters);

    // Publish clustered clouds
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*clustered_cloud_ptr, cloud_msg);
    cloud_msg.header = _sensor_header;
    _pub_cluster_cloud->publish(cloud_msg);
    
    // Publish clusters to MOOS
    sendToGateway(cloud_clusters);

    // Publish matched cloud clusters
    cloud_clusters.header = _sensor_header;
    _pub_matched_clusters_msg->publish(cloud_clusters);

    // Publish visualization markers
    if (_viz) markers(cloud_clusters);
}

void ClusterNode::odomCallback(const nav_msgs::msg::Odometry &msg)
{
    nav_x_ = msg.pose.pose.position.x;
    nav_y_ = msg.pose.pose.position.y;
    tf2::Quaternion q;
    q.setW(msg.pose.pose.orientation.w);
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    nav_heading_ = y;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
