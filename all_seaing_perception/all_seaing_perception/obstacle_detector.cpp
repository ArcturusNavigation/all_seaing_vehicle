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
    this->declare_parameter<std::string>("global_frame_id", "odom");
    this->declare_parameter<int>("obstacle_size_min", 20);
    this->declare_parameter<int>("obstacle_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("polygon_area_thresh", 100000.0);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
    m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
    m_clustering_distance = this->get_parameter("clustering_distance").as_double();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_polygon_area_thresh = this->get_parameter("polygon_area_thresh").as_double();

    // Initialize navigation variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_heading = 0;

    // Initialize publishers and subscribers
    m_raw_map_pub =
        this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/raw", 10);
    m_unlabeled_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/unlabeled", 10);
    m_gateway_pub =
        this->create_publisher<protobuf_client_interfaces::msg::Gateway>("/send_to_gateway", 10);
    m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "point_cloud", rclcpp::SensorDataQoS(),
        std::bind(&ObstacleDetector::pc_callback, this, std::placeholders::_1));
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&ObstacleDetector::odom_callback, this, std::placeholders::_1));
}

std::vector<std::shared_ptr<all_seaing_perception::Obstacle>>
ObstacleDetector::cluster_cloud(builtin_interfaces::msg::Time current_time,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr) {

    // Flatten input point cloud to 2D
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
    for (size_t i = 0; i < cloud_2d->points.size(); i++)
        cloud_2d->points[i].z = 0;

    // Extract clusters from point cloud and save indices in obstacle_indices
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
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
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> obstacles;
    for (auto it = obstacles_indices.begin(); it != obstacles_indices.end(); it++) {
        std::shared_ptr<all_seaing_perception::Obstacle> obstacle(
            new all_seaing_perception::Obstacle(in_cloud_ptr, it->indices, m_obstacle_id++,
                                                current_time, m_nav_x, m_nav_y, m_nav_heading));
        obstacles.push_back(obstacle);
    }

    return obstacles;
}

void ObstacleDetector::track_obstacles(
    builtin_interfaces::msg::Time current_time,
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &raw_obstacles) {

    // Remove tracked obstacles if haven't seen for obstacle drop threshold
    for (auto it = m_tracked_obstacles.begin(); it != m_tracked_obstacles.end(); it++) {
        rclcpp::Duration obstacle_last_seen((*it)->get_last_seen().sec,
                                            (*it)->get_last_seen().nanosec);
        rclcpp::Duration missing_duration =
            rclcpp::Duration(current_time.sec, current_time.nanosec) - obstacle_last_seen;

        if (missing_duration.seconds() >= m_obstacle_drop_thresh) {
            it = m_tracked_obstacles.erase(it);
            it--;
        }
    }

    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> new_obstacles;
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
        for (unsigned long j = 0; j < m_tracked_obstacles.size(); j++) {
            // Skip indices already chosen
            if (chosen_indices.find(j) != chosen_indices.end())
                continue;

            float curr_dist = pcl::euclideanDistance(m_tracked_obstacles[j]->get_global_point(),
                                                     raw_obstacle->get_global_point());
            if (curr_dist < best_dist) {
                best_match = j;
                best_dist = curr_dist;
            }
        }

        if (best_match >= 0) {
            // If there is best match, then set ID, move to tracked obstacles, and
            // remember chosen index
            raw_obstacle->set_id(m_tracked_obstacles[best_match]->get_id());
            m_tracked_obstacles[best_match] = raw_obstacle;
            chosen_indices.insert(best_match);
        } else {
            // If there are no matches, then add to new obstacles
            new_obstacles.push_back(raw_obstacle);
        }
    }

    // Update tracked_obstacles
    if (m_tracked_obstacles.empty())
        m_tracked_obstacles = std::move(raw_obstacles);
    else
        m_tracked_obstacles.insert(m_tracked_obstacles.end(), new_obstacles.begin(),
                                   new_obstacles.end());
}

void ObstacleDetector::send_to_gateway(const all_seaing_interfaces::msg::ObstacleMap &in_map) {
    for (const auto &obstacle : in_map.obstacles) {
        for (const auto &p : obstacle.global_chull.polygon.points) {
            auto gateway_msg = protobuf_client_interfaces::msg::Gateway();
            gateway_msg.gateway_key = "TRACKED_FEATURE";
            gateway_msg.gateway_string = "x=" + std::to_string(p.x) + ",y=" + std::to_string(p.y) +
                                         ",label=" + std::to_string(obstacle.id);
            m_gateway_pub->publish(gateway_msg);
        }
    }
}

// Main callback loop
void ObstacleDetector::pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud) {

    // Convert ROS2 PointCloud2 to pcl PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud, *cloud_ptr);

    // Cluster the pointcloud by the distance of the points
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> raw_obstacles =
        cluster_cloud(in_cloud->header.stamp, cloud_ptr);

    // Match raw obstacles and tracked obstacles
    track_obstacles(in_cloud->header.stamp, raw_obstacles);

    // Create global header
    std_msgs::msg::Header global_header = std_msgs::msg::Header();
    global_header.frame_id = m_global_frame_id;
    global_header.stamp = in_cloud->header.stamp;

    // Publish raw obstacle map
    all_seaing_interfaces::msg::ObstacleMap raw_map;
    raw_map.ns = "raw";
    raw_map.local_header = in_cloud->header;
    raw_map.header = global_header;
    for (unsigned int i = 0; i < raw_obstacles.size(); i++) {
        all_seaing_interfaces::msg::Obstacle raw_obstacle;
        raw_obstacles[i]->to_ros_msg(in_cloud->header, global_header, raw_obstacle);
        raw_map.obstacles.push_back(raw_obstacle);
    }
    m_raw_map_pub->publish(raw_map);

    // Publish tracked obstacle map
    all_seaing_interfaces::msg::ObstacleMap tracked_map;
    tracked_map.ns = "unlabeled";
    tracked_map.local_header = in_cloud->header;
    tracked_map.header = global_header;
    for (unsigned int i = 0; i < m_tracked_obstacles.size(); i++) {
        all_seaing_interfaces::msg::Obstacle tracked_obstacle;
        m_tracked_obstacles[i]->to_ros_msg(in_cloud->header, global_header, tracked_obstacle);
        tracked_map.obstacles.push_back(tracked_obstacle);
    }
    m_unlabeled_map_pub->publish(tracked_map);

    // Send obstacles to MOOS
    send_to_gateway(tracked_map);
}

// TODO: this can be removed after using TF rather than calculating by ourselves
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
