#include "all_seaing_perception/obstacle_detector.hpp"

#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

ObstacleDetector::ObstacleDetector() : Node("obstacle_detector") {
    // Initialize parameters
    this->declare_parameter<int>("obstacle_size_min", 20);
    this->declare_parameter<int>("obstacle_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("polygon_area_thresh", 100000.0);
    this->declare_parameter<bool>("viz", true);

    // Initialize member variables from parameters
    m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
    m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
    m_clustering_distance = this->get_parameter("clustering_distance").as_double();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_polygon_area_thresh = this->get_parameter("polygon_area_thresh").as_double();
    m_viz = this->get_parameter("viz").as_bool();

    // Initialize navigation variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_heading = 0;

    // Initialize publishers and subscribers
    m_obstacle_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud/obstacles", 10);
    m_raw_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/raw", 10);
    m_unlabeled_map_pub =
        this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
            "obstacle_map/unlabeled", 10);
    m_marker_array_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "chull_markers/unlabeled", 10);
    m_text_marker_array_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "text_markers/unlabeled", 10);
    m_gateway_pub = this->create_publisher<protobuf_client_interfaces::msg::Gateway>(
        "/send_to_gateway", 10);
    m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SensorDataQoS(),
        std::bind(&ObstacleDetector::pc_callback, this, std::placeholders::_1));
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&ObstacleDetector::odom_callback, this, std::placeholders::_1));
}

void ObstacleDetector::segment_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    all_seaing_interfaces::msg::ObstacleMap &out_map) {
    // Cluster the pointcloud by the distance of the points
    std::vector<std::shared_ptr<Obstacle>> raw_obstacles;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>(*in_cloud_ptr));
    raw_obstacles = cluster_cloud(cloud_ptr);

    // Match raw obstacles and tracked obstacles
    match_obstacles(raw_obstacles, m_tracked_obstacles);

    // Push raw obstacles to ObstacleMap and publish
    all_seaing_interfaces::msg::ObstacleMap raw_map;
    for (unsigned int i = 0; i < raw_obstacles.size(); i++) {
        all_seaing_interfaces::msg::Obstacle raw_obstacle;
        raw_obstacles[i]->to_ros_msg(m_lidar_header, raw_obstacle);
        raw_map.obstacles.push_back(raw_obstacle);
    }
    m_raw_map_pub->publish(raw_map);

    // Final pointcloud to be published
    for (unsigned int i = 0; i < m_tracked_obstacles.size(); i++) {
        *out_cloud_ptr += *m_tracked_obstacles[i]->get_cloud();

        // Push matched obstacles to ObstacleMap
        all_seaing_interfaces::msg::Obstacle tracked_obstacle;
        m_tracked_obstacles[i]->to_ros_msg(m_lidar_header, tracked_obstacle);
        out_map.obstacles.push_back(tracked_obstacle);
    }
}

std::vector<std::shared_ptr<Obstacle>> ObstacleDetector::cluster_cloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr) {
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZI>());

    // Flatten input pointcloud to 2D
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    // Extract clusters from point cloud and save indices in obstacle_indices
    if (!cloud_2d->points.empty())
        tree->setInputCloud(cloud_2d);
    std::vector<pcl::PointIndices> obstacles_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(m_clustering_distance);
    ec.setMinClusterSize(m_obstacle_size_min);
    ec.setMaxClusterSize(m_obstacle_size_max);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_2d);
    ec.extract(obstacles_indices);

    // Cluster points into obstacles
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    for (auto it = obstacles_indices.begin(); it != obstacles_indices.end(); it++) {
        std::shared_ptr<Obstacle> obstacle(
            new Obstacle(in_cloud_ptr, it->indices, m_lidar_header, m_obstacle_id++,
                         m_current_time, m_nav_x, m_nav_y, m_nav_heading));
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

void ObstacleDetector::match_obstacles(
    std::vector<std::shared_ptr<Obstacle>> &raw_obstacles,
    std::vector<std::shared_ptr<Obstacle>> &tracked_obstacles) {
    // Remove tracked obstacles if haven't seen for obstacle drop threshold
    for (auto it = tracked_obstacles.begin(); it != tracked_obstacles.end(); it++) {
        rclcpp::Duration current_time(m_current_time.sec, m_current_time.nanosec);
        rclcpp::Duration obstacle_last_seen((*it)->get_last_seen().sec,
                                            (*it)->get_last_seen().nanosec);
        rclcpp::Duration missing_duration = current_time - obstacle_last_seen;

        if (missing_duration.seconds() >= m_obstacle_drop_thresh) {
            it = tracked_obstacles.erase(it);
            it--;
        }
    }

    std::vector<std::shared_ptr<Obstacle>> new_obstacles;
    std::unordered_set<int> chosen_indices;
    for (auto it = raw_obstacles.begin(); it != raw_obstacles.end(); it++) {
        auto &raw_obstacle = *it;

        // Remove obstacle if larger than polygon area threshold
        if (raw_obstacle->get_polygon_area() > m_polygon_area_thresh) {
            it = raw_obstacles.erase(it);
            it--;
            continue;
        }

        // Match obstacles based on closest average point
        float best_dist = m_obstacle_seg_thresh;
        int best_match = -1;
        for (unsigned long j = 0; j < tracked_obstacles.size(); j++) {
            // Skip indices already chosen
            if (chosen_indices.find(j) != chosen_indices.end())
                continue;

            float curr_dist =
                pcl::euclideanDistance(tracked_obstacles[j]->get_global_point(),
                                       raw_obstacle->get_global_point());
            if (curr_dist < best_dist) {
                best_match = j;
                best_dist = curr_dist;
            }
        }

        if (best_match >= 0) {
            // If there is best match, then set ID, move to tracked obstacles, and
            // remember chosen index
            raw_obstacle->set_id(tracked_obstacles[best_match]->get_id());
            tracked_obstacles[best_match] = raw_obstacle;
            chosen_indices.insert(best_match);
        } else {
            // If there are no matches, then add to new obstacles
            new_obstacles.push_back(raw_obstacle);
        }
    }

    // Update tracked_obstacles
    if (tracked_obstacles.empty())
        tracked_obstacles = std::move(raw_obstacles);
    else
        tracked_obstacles.insert(tracked_obstacles.end(), new_obstacles.begin(),
                                 new_obstacles.end());
}

void ObstacleDetector::markers(const all_seaing_interfaces::msg::ObstacleMap &in_map) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray text_marker_array;

    for (const auto &obstacle : in_map.obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "odom";
        marker.ns = "unlabeled_obstacle";
        marker.id = obstacle.id;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        marker.pose.position.x = obstacle.global_point.x;
        marker.pose.position.y = obstacle.global_point.y;
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.header.stamp = this->get_clock()->now();
        text_marker.header.frame_id = "odom";
        text_marker.ns = "unlabeled_text";
        text_marker.id = obstacle.id;
        text_marker.scale.z = 0.7; // Text scale in RVIZ
        text_marker.color.a = 1.0;
        text_marker.color.g = 1.0;
        text_marker.lifetime = rclcpp::Duration::from_seconds(1.3);
        text_marker.text = std::to_string(obstacle.id);
        text_marker.pose.position.x = obstacle.global_point.x;
        text_marker.pose.position.y = obstacle.global_point.y;
        text_marker.pose.position.z = 1.0;
        text_marker_array.markers.push_back(text_marker);
    }

    m_marker_array_pub->publish(marker_array);
    m_text_marker_array_pub->publish(text_marker_array);
}

void ObstacleDetector::send_to_gateway(
    const all_seaing_interfaces::msg::ObstacleMap &in_map) {
    for (const auto &obstacle : in_map.obstacles) {
        for (const auto &p : obstacle.global_chull.points) {
            auto gateway_msg = protobuf_client_interfaces::msg::Gateway();
            gateway_msg.gateway_key = "TRACKED_FEATURE";
            gateway_msg.gateway_string = "x=" + std::to_string(p.x) +
                                         ",y=" + std::to_string(p.y) +
                                         ",label=" + std::to_string(obstacle.id);
            m_gateway_pub->publish(gateway_msg);
        }
    }
}

// Main callback loop
void ObstacleDetector::pc_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud) {
    // Set header and timestamp
    m_current_time = in_cloud->header.stamp;
    m_lidar_header = in_cloud->header;

    // Convert ROS2 PointCloud2 to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud, *current_cloud_ptr);

    // Segment cloud into clustered obstacles
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    all_seaing_interfaces::msg::ObstacleMap obstacle_map;
    segment_cloud(current_cloud_ptr, clustered_cloud_ptr, obstacle_map);

    // Publish clustered clouds
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*clustered_cloud_ptr, cloud_msg);
    cloud_msg.header = m_lidar_header;
    m_obstacle_cloud_pub->publish(cloud_msg);

    // Publish obstacles to MOOS
    send_to_gateway(obstacle_map);

    // Publish matched obstacle map
    obstacle_map.header = m_lidar_header;
    m_unlabeled_map_pub->publish(obstacle_map);

    // Publish visualization markers
    if (m_viz)
        markers(obstacle_map);
}

void ObstacleDetector::odom_callback(const nav_msgs::msg::Odometry &msg) {
    m_nav_x = msg.pose.pose.position.x;
    m_nav_y = msg.pose.pose.position.y;
    tf2::Quaternion q;
    q.setW(msg.pose.pose.orientation.w);
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    m_nav_heading = y;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
