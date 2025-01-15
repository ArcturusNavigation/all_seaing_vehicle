#include "pcl/common/point_tests.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter() : Node("point_cloud_filter") {
        // Initialize parameters
        this->declare_parameter<double>("range_min_threshold", 0.0);
        this->declare_parameter<double>("range_max_threshold", 100000.0);
        this->declare_parameter<double>("intensity_low_threshold", 0.0);
        this->declare_parameter<double>("intensity_high_threshold", 100000.0);
        this->declare_parameter<double>("leaf_size", 0.0);

        // Subscribe to the input point cloud topic
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFilter::pc_callback, this, std::placeholders::_1));

        // Advertise the filtered point cloud topic
        m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud/filtered", rclcpp::SensorDataQoS());

        // Get values from parameter server
        m_range_min_threshold = this->get_parameter("range_min_threshold").as_double();
        m_range_max_threshold = this->get_parameter("range_max_threshold").as_double();
        m_intensity_low_threshold = this->get_parameter("intensity_low_threshold").as_double();
        m_intensity_high_threshold = this->get_parameter("intensity_high_threshold").as_double();
        m_leaf_size = this->get_parameter("leaf_size").as_double();
    }

private:
    void downsample_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(in_cloud_ptr);
        vg.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
        vg.filter(*out_cloud_ptr);
    }

    void filter_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr) {
        for (const auto &point : in_cloud_ptr->points) {
            double range = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            // Keep point if in range thresholds, intensity threshold, and is finite
            if (m_range_min_threshold <= range && range <= m_range_max_threshold &&
                m_intensity_low_threshold <= point.intensity &&
                point.intensity <= m_intensity_high_threshold &&
                pcl::isFinite(point)) {
                out_cloud_ptr->points.push_back(point);
            }
        }
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2 &in_cloud_msg) {
        // Transform in_cloud_msg and convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(in_cloud_msg, *in_cloud_ptr);

        // Filter out points ouside intensity/distance range, and NaN values
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
        filter_cloud(in_cloud_ptr, filtered_cloud_ptr);

        // Downsample cloud if leaf size is not equal to 0
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
        if (m_leaf_size != 0)
            downsample_cloud(filtered_cloud_ptr, downsampled_cloud_ptr);
        else
            downsampled_cloud_ptr = filtered_cloud_ptr;

        // Convert filtered point cloud back to ROS message
        sensor_msgs::msg::PointCloud2 new_cloud_msg;
        pcl::toROSMsg(*downsampled_cloud_ptr, new_cloud_msg);
        new_cloud_msg.header = in_cloud_msg.header;
        m_publisher->publish(new_cloud_msg);
    }

    // Subscribers, publishers, and member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
    double m_range_min_threshold;
    double m_range_max_threshold;
    double m_intensity_low_threshold;
    double m_intensity_high_threshold;
    double m_leaf_size;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
