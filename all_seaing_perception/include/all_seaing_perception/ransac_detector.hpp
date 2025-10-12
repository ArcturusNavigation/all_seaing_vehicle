#ifndef ALL_SEAING_PERCEPTION__RANSAC_DETECTOR_HPP
#define ALL_SEAING_PERCEPTION__RANSAC_DETECTOR_HPP

#include <string>
#include <vector>
#include <algorithm>
#include <exception>
#include <cmath>
#include <chrono>
#include <exception>
#include <thread>
#include <set>

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h" 

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include "cv_bridge/cv_bridge.h"

// #include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "all_seaing_interfaces/msg/labeled_object_point_cloud_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud.hpp"
#include "all_seaing_interfaces/msg/labeled_object_plane_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_plane.hpp"

#include "all_seaing_perception/perception_utilities.hpp"

class RANSACDetector : public rclcpp::Node{
private:
    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::LabeledObjectPlaneArray>::SharedPtr m_object_ransac_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_ransac_viz_pub;
    rclcpp::Subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>::SharedPtr m_object_pcl_sub;

    // Get the object point clouds, filter the ones that represent banners, and do RANSAC on those
    void object_pcl_cb(
        const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &in_pcl_msg);

    // Converts the output of the RANSAC utility function to the desired msg
    all_seaing_interfaces::msg::LabeledObjectPlane to_plane_msg(int label, Eigen::Vector3d centroid, Eigen::Matrix3d normal, Eigen::Vector3d size);

    visualization_msgs::msg::MarkerArray visualize_plane(all_seaing_interfaces::msg::LabeledObjectPlane msg, int& marker_id, double r = 0, double g = 0, double b = 1);

    visualization_msgs::msg::MarkerArray visualize_planes(all_seaing_interfaces::msg::LabeledObjectPlaneArray msg);

    // member variables
    std::string m_ransac_params_file, m_label_mappings_file;

    YAML::Node m_ransac_params_config_yaml, m_label_mappings_config_yaml;

    std::set<std::string> m_labels;
    std::map<int, std::string> m_id_label_map;
    std::map<std::string, int> m_label_id_map;
    std::map<std::string, std::vector<int>> m_coplanar_id;
    std_msgs::msg::Header m_header;

    // ransac params
    int m_max_iters;
    double m_dist_thres;
    int m_min_inliers;
    double m_clust_dist;

public:
    RANSACDetector();
    virtual ~RANSACDetector();
};

#endif // ALL_SEAING_PERCEPTION__RANSAC_DETECTOR_HPP