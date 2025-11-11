#include "pcl/common/point_tests.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "all_seaing_perception/perception_utilities.hpp"

#include <string>
#include <vector>

class PointCloudFilter : public rclcpp::Node {
public:
    PointCloudFilter() : Node("point_cloud_filter") {
        // Initialize parameters
        this->declare_parameter<std::string>("robot_frame_id", "base_link");
        this->declare_parameter<std::string>("global_frame_id", "map");
        this->declare_parameter<std::vector<double>>("range_radius", {0.0, 100000.0});
        this->declare_parameter<std::vector<double>>("range_intensity", {0.0, 100000.0});
        this->declare_parameter<std::vector<double>>("range_x", {-100000.0, 100000.0});
        this->declare_parameter<std::vector<double>>("range_y", {-100000.0, 100000.0});
        this->declare_parameter<std::vector<double>>("range_z", {-100000.0, 100000.0});
        this->declare_parameter<std::vector<double>>("local_range_x", {-100000.0, 100000.0});
        this->declare_parameter<std::vector<double>>("local_range_y", {-100000.0, 100000.0});
        this->declare_parameter<std::vector<double>>("local_range_z", {-100000.0, 100000.0});
        this->declare_parameter<double>("leaf_size_xy", 0.0);
        this->declare_parameter<double>("leaf_size_z", 0.0);
        this->declare_parameter<int>("min_pts_per_voxel", 1);
        this->declare_parameter<bool>("convert_to_robot", false);

        m_got_lidar_to_robot_tf = false;

        // Initialize tf_listener pointer
        m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

        // Subscribe to the input point cloud topic
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS(),
            std::bind(&PointCloudFilter::pc_callback, this, std::placeholders::_1));

        // Advertise the filtered point cloud topic
        // m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     "point_cloud/filtered", rclcpp::SensorDataQoS());
        m_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "point_cloud/filtered", rclcpp::SensorDataQoS().reliable());

        // Get values from parameter server
        m_robot_frame_id = this->get_parameter("robot_frame_id").as_string();
        m_global_frame_id = this->get_parameter("global_frame_id").as_string();
        m_range_radius = this->get_parameter("range_radius").as_double_array();
        m_range_intensity = this->get_parameter("range_intensity").as_double_array();
        m_range_x = this->get_parameter("range_x").as_double_array();
        m_range_y = this->get_parameter("range_y").as_double_array();
        m_range_z = this->get_parameter("range_z").as_double_array();
        m_local_range_x = this->get_parameter("local_range_x").as_double_array();
        m_local_range_y = this->get_parameter("local_range_y").as_double_array();
        m_local_range_z = this->get_parameter("local_range_z").as_double_array();
        m_leaf_size_xy = this->get_parameter("leaf_size_xy").as_double();
        m_leaf_size_z = this->get_parameter("leaf_size_z").as_double();
        m_min_pts_per_voxel = this->get_parameter("min_pts_per_voxel").as_int();
        m_convert_to_robot = this->get_parameter("convert_to_robot").as_bool();
    }

private:
    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame, bool convert_to_robot = false) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
            if(convert_to_robot){
                RCLCPP_INFO(this->get_logger(), "GOT LIDAR->BASE_LINK TF");
                m_got_lidar_to_robot_tf = true;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        return tf;
    }

    void downsample_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr) {
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(in_cloud_ptr);
        vg.setLeafSize(m_leaf_size_xy, m_leaf_size_xy, m_leaf_size_z);
        vg.setMinimumPointsNumberPerVoxel(m_min_pts_per_voxel);
        vg.filter(*out_cloud_ptr);
    }

    void filter_cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud_ptr) {
        // TODO use PCL functions instead, to also not have issues with the fields not being copied if we don't do that manually
        // Keep point if in radius thresholds, intensity threshold, and is finite
        for (const auto &point : in_cloud_ptr->points) {
            double radius = sqrt(point.x * point.x + point.y * point.y);

            // // Convert point to map
            // geometry_msgs::msg::Point point_msg;
            // geometry_msgs::msg::Point point_tf;
            // point_msg.x = point.x;
            // point_msg.y = point.y;
            // point_msg.z = point.z;
            // tf2::doTransform<geometry_msgs::msg::Point>(point_msg, point_tf, m_lidar_map_tf);

            // if (m_range_x[0] <= point_tf.x && point_tf.x <= m_range_x[1] &&
            //     m_range_y[0] <= point_tf.y && point_tf.y <= m_range_y[1] &&
            //     m_range_z[0] <= point_tf.z && point_tf.z <= m_range_z[1] &&
            if (m_range_radius[0] <= radius && radius <= m_range_radius[1] &&
                m_local_range_x[0] <= point.x && point.x <= m_local_range_x[1] &&
                m_local_range_y[0] <= point.y && point.y <= m_local_range_y[1] &&
                m_local_range_z[0] <= point.z && point.z <= m_local_range_z[1] &&
                m_range_intensity[0] <= point.intensity &&
                point.intensity <= m_range_intensity[1] && 
                pcl::isFinite(point)) {
                out_cloud_ptr->push_back(point);
            }
        }
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2 &in_cloud_msg) {
        // m_lidar_map_tf = get_tf(m_global_frame_id, in_cloud_msg.header.frame_id);
        
        if (m_convert_to_robot && !m_got_lidar_to_robot_tf){
            m_lidar_robot_tf = get_tf(m_robot_frame_id, in_cloud_msg.header.frame_id, true);
            if (!m_got_lidar_to_robot_tf)
                return;
        }

        // Convert to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(in_cloud_msg, *in_cloud_ptr);

        // Filter out points ouside intensity/distance radius, and NaN values
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
        filter_cloud(in_cloud_ptr, filtered_cloud_ptr);

        // Downsample cloud if leaf size is not equal to 0
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZI>);
        if (m_leaf_size_xy > 0)
            downsample_cloud(filtered_cloud_ptr, downsampled_cloud_ptr);
        else
            downsampled_cloud_ptr = filtered_cloud_ptr;

        // Convert filtered point cloud back to ROS message
        sensor_msgs::msg::PointCloud2 new_cloud_msg;
        if (m_convert_to_robot){
            pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud_ptr(
                new pcl::PointCloud<pcl::PointXYZI>);
            if (downsampled_cloud_ptr->size() != 0){
                all_seaing_perception::transformPCLCloud(*downsampled_cloud_ptr, *tf_cloud_ptr, m_lidar_robot_tf);
            }
            pcl::toROSMsg(*tf_cloud_ptr, new_cloud_msg);
            new_cloud_msg.header = in_cloud_msg.header;
            new_cloud_msg.header.frame_id = m_robot_frame_id;
        }else{
            pcl::toROSMsg(*downsampled_cloud_ptr, new_cloud_msg);
            new_cloud_msg.header = in_cloud_msg.header;
        }
        m_cloud_pub->publish(new_cloud_msg);
    }

    // Subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;

    // Transform variables
    // geometry_msgs::msg::TransformStamped m_lidar_map_tf;
    geometry_msgs::msg::TransformStamped m_lidar_robot_tf;
    bool m_got_lidar_to_robot_tf;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    // Member variables
    std::string m_robot_frame_id;
    std::string m_global_frame_id;
    std::vector<double> m_range_radius;
    std::vector<double> m_range_intensity;
    std::vector<double> m_range_x;
    std::vector<double> m_range_y;
    std::vector<double> m_range_z;
    std::vector<double> m_local_range_x;
    std::vector<double> m_local_range_y;
    std::vector<double> m_local_range_z;
    double m_leaf_size_xy, m_leaf_size_z;
    int m_min_pts_per_voxel;
    bool m_convert_to_robot;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
