#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "all_seaing_perception/obstacle.hpp"

#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <string>
#include <vector>

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector() : Node("obstacle_detector") {

        // Initialize parameters
        this->declare_parameter<std::string>("global_frame_id", "map");
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

        // Initialize tf_listener pointer
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        // Initialize publishers and subscribers
        m_raw_map_pub =
            this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/raw", 10);
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleDetector::pc_callback, this, std::placeholders::_1));
    }

private:
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>>
    cluster_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr) {

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
                new all_seaing_perception::Obstacle(m_local_header, m_global_header, in_cloud_ptr,
                                                    it->indices, m_obstacle_id++, m_lidar_map_tf));
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }

    void publish_map(const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map) {
        all_seaing_interfaces::msg::ObstacleMap map_msg;
        map_msg.ns = "raw";
        map_msg.local_header = m_local_header;
        map_msg.header = m_global_header;
        map_msg.is_labeled = false;
        for (unsigned int i = 0; i < map.size(); i++) {
            all_seaing_interfaces::msg::Obstacle raw_obstacle;
            map[i]->to_ros_msg(raw_obstacle);
            map_msg.obstacles.push_back(raw_obstacle);
        }
        m_raw_map_pub->publish(map_msg);
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud) {

        // Set up headers and transforms
        m_local_header = in_cloud->header;
        m_global_header.frame_id = m_global_frame_id;
        m_global_header.stamp = m_local_header.stamp;
        m_lidar_map_tf = get_tf(m_global_frame_id, m_local_header.frame_id);

        // Convert ROS2 PointCloud2 to pcl PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*in_cloud, *cloud_ptr);

        // Cluster the pointcloud by the distance of the points
        std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> raw_obstacles =
            cluster_cloud(cloud_ptr);

        // Publish raw map
        publish_map(raw_obstacles);
    }

    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        return tf;
    }

    // Member variables
    std::string m_global_frame_id;
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_obstacle_id;
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    double m_clustering_distance;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_polygon_area_thresh;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_lidar_map_tf;

    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_raw_map_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
