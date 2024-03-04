#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/point_tests.h"

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("point_cloud_filter")
    {
        // Initialize parameters
        this->declare_parameter<double>("intensity_low_threshold", 0.0);
        this->declare_parameter<double>("intensity_high_threshold", 50.0);

        // Subscribe to the input point cloud topic
        m_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/in_cloud", 10, std::bind(&PointCloudFilter::pc_callback, this, std::placeholders::_1));

        // Advertise the filtered point cloud topic
        m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/out_cloud", 10);

        // Get intensity thresholds from parameter server
        m_intensity_low_threshold = this->get_parameter("intensity_low_threshold").as_double();
        m_intensity_high_threshold = this->get_parameter("intensity_high_threshold").as_double();
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2 &in_cloud_msg)
    {
        // Transform in_cloud_msg and convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI> in_cloud;
        pcl::fromROSMsg(in_cloud_msg, in_cloud);

        // Iterate through in_cloud and filter out points ouside intensity range
        pcl::PointCloud<pcl::PointXYZI> new_cloud;
        for (pcl::PointXYZI &pt : in_cloud.points)
        {
            if (m_intensity_low_threshold <= pt.intensity && pt.intensity <= m_intensity_high_threshold && pcl::isFinite(pt))
            {
                new_cloud.points.push_back(pt);
            }
        }
        
        // Convert filtered point cloud back to ROS message
        sensor_msgs::msg::PointCloud2 new_cloud_msg;
        pcl::toROSMsg(new_cloud, new_cloud_msg);
        new_cloud_msg.header = in_cloud_msg.header;
        m_publisher->publish(new_cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
    double m_intensity_low_threshold;
    double m_intensity_high_threshold;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
