#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "all_seaing_perception/obstacle.hpp"
#include "all_seaing_perception/perception_utilities.hpp"

#include <pcl/common/distances.h>
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <string>
#include <vector>
#include <chrono>

class ObstacleDetector : public rclcpp::Node {
public:
    ObstacleDetector() : Node("obstacle_detector") {

        // Initialize parameters
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<std::string>("global_frame_id", "map");
        this->declare_parameter<double>("clustering_distance", 0.75);
        this->declare_parameter<int>("obstacle_size_min", 20);
        this->declare_parameter<int>("obstacle_size_max", 100000);
        this->declare_parameter<int>("obstacle_filter_pts_max", 100000);
        this->declare_parameter<double>("obstacle_filter_area_max", 100000.0);
        this->declare_parameter<double>("obstacle_filter_length_max", 100000.0);
        this->declare_parameter<double>("range_max", 200.0);
        this->declare_parameter<bool>("flatten", false);

        // Initialize member variables from parameters
        m_base_link_frame = this->get_parameter("base_link_frame").as_string();
        m_global_frame_id = this->get_parameter("global_frame_id").as_string();
        m_clustering_distance = this->get_parameter("clustering_distance").as_double();
        m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
        m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
        m_obstacle_filter_size_max = this->get_parameter("obstacle_filter_pts_max").as_int();
        m_obstacle_filter_area_max = this->get_parameter("obstacle_filter_area_max").as_double();
        m_obstacle_filter_length_max = this->get_parameter("obstacle_filter_length_max").as_double();
        m_range_max = this->get_parameter("range_max").as_double();
        m_flatten = this->get_parameter("flatten").as_bool();

        // Initialize tf_listener pointer
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        // Initialize publishers and subscribers
        m_raw_map_pub =
            this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/raw", 10);
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&ObstacleDetector::pc_callback, this, std::placeholders::_1));

        m_got_lidar_base_link_tf = false;
    }

private:
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle<pcl::PointXYZI>>>
    cluster_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr) {

        // Flatten input point cloud to 2D
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
        if (m_flatten){
            for (size_t i = 0; i < cloud_2d->points.size(); i++){
                cloud_2d->points[i].z = 0;
            }
        }

        // Extract clusters from point cloud and save indices in obstacle_indices
        std::vector<pcl::PointIndices> obstacles_indices;
        all_seaing_perception::euclideanClustering(cloud_2d, obstacles_indices, m_clustering_distance, m_obstacle_size_min, m_obstacle_size_max);

        // Cluster points into obstacles
        std::vector<std::shared_ptr<all_seaing_perception::Obstacle<pcl::PointXYZI>>> obstacles;
        for (auto it = obstacles_indices.begin(); it != obstacles_indices.end(); it++) {
            // Filter out large obstacles
            if (it->indices.size() > m_obstacle_filter_size_max){
                continue;
            }
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lidar_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*in_cloud_ptr, it->indices, *cloud_lidar_ptr);
            // Transform point cloud to base link
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base_link_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            all_seaing_perception::transformPCLCloud(*cloud_lidar_ptr, *cloud_base_link_ptr, m_lidar_base_link_tf);
            
            // perform PCA and filter based on largest dimension, to only get point-like detections and not edges or anything
            Eigen::Vector4d centr;
            Eigen::Matrix3d eigenvecs;
            Eigen::Vector3d axes_length;            
            std::tie(centr, eigenvecs, axes_length) = all_seaing_perception::minimumOrientedBBox(*cloud_base_link_ptr);

            if (axes_length[2] > m_obstacle_filter_length_max){
                continue;
            }

            // Publish global raw map (for obstacle avoidance)
            std::vector<int> ind(it->indices.size(), 0);
            std::iota(ind.begin(), ind.end(), 0);
            std::shared_ptr<all_seaing_perception::Obstacle<pcl::PointXYZI>> obstacle(
                new all_seaing_perception::Obstacle<pcl::PointXYZI>(m_local_header, m_global_header, cloud_base_link_ptr,
                                                    ind, m_obstacle_id++, m_base_link_map_tf));
            // Filter out obstacles far apart (probably random stuff/shore)
            pcl::PointXYZI obs_ctr = obstacle->get_local_point();
            pcl::PointXYZI orig(0, 0, 0);
            if (pcl::euclideanDistance(orig, obs_ctr) > m_range_max || obstacle->get_polygon_area()>m_obstacle_filter_area_max){
                continue;
            }
            obstacles.push_back(obstacle);
        }
        return obstacles;
    }
    
    void publish_map(const std::vector<std::shared_ptr<all_seaing_perception::Obstacle<pcl::PointXYZI>>> &map) {
        all_seaing_interfaces::msg::ObstacleMap map_msg;
        map_msg.ns = "raw";
        map_msg.local_header = m_local_header;
        map_msg.header = m_global_header;
        map_msg.is_labeled = false;
        for (unsigned int i = 0; i < map.size(); i++) {
            all_seaing_interfaces::msg::Obstacle raw_obstacle;
            map[i]->to_ros_msg(raw_obstacle);
            raw_obstacle.label = -1;
            map_msg.obstacles.push_back(raw_obstacle);
        }
        m_raw_map_pub->publish(map_msg);
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud) {
        // // measuring time
        // using std::chrono::high_resolution_clock;
        // using std::chrono::duration_cast;
        // using std::chrono::duration;
        // using std::chrono::milliseconds;

        // Set up headers and transforms
        m_local_header = in_cloud->header;
        m_global_header.frame_id = m_global_frame_id;
        m_global_header.stamp = m_local_header.stamp;
        if (!m_got_lidar_base_link_tf){
            m_lidar_base_link_tf = get_tf(m_base_link_frame, m_local_header.frame_id, "lidar_base_link");
        }
        m_local_header.frame_id = m_base_link_frame;
        m_base_link_map_tf = get_tf(m_global_frame_id, m_base_link_frame, "base_link_map");

        // Convert ROS2 PointCloud2 to pcl PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*in_cloud, *cloud_ptr);

        // auto t1 = high_resolution_clock::now();
        // Cluster the pointcloud by the distance of the points
        std::vector<std::shared_ptr<all_seaing_perception::Obstacle<pcl::PointXYZI>>> raw_obstacles =
            cluster_cloud(cloud_ptr);
        // auto t2 = high_resolution_clock::now();

        // /* Getting number of milliseconds as a double. */
        // duration<double, std::milli> ms_double = t2 - t1;

        // RCLCPP_INFO(this->get_logger(), "# POINTS: %d, CLUSTERING TIME: %lfms", cloud_ptr->points.size(), ms_double.count());

        // Publish raw map
        publish_map(raw_obstacles);
    }

    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame, std::string info) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
            // RCLCPP_INFO(this->get_logger(), "Transform %s to %s", in_src_frame.c_str(), in_target_frame.c_str());
            if (info == "lidar_base_link"){
                m_got_lidar_base_link_tf = true;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        return tf;
    }

    // Member variables
    std::string m_base_link_frame, m_global_frame_id;
    std_msgs::msg::Header m_local_header;
    std_msgs::msg::Header m_global_header;
    int m_obstacle_id;
    double m_clustering_distance;
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    int m_obstacle_filter_size_max;
    double m_obstacle_filter_area_max;
    double m_obstacle_filter_length_max;
    double m_range_max;
    bool m_flatten;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_lidar_base_link_tf, m_base_link_map_tf;
    bool m_got_lidar_base_link_tf;

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
