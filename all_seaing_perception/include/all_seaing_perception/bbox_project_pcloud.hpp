#ifndef ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP
#define ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP

#include <string>
#include <vector>
#include <algorithm>
#include <exception>
#include <cmath>

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h" 

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include "pcl_conversions/pcl_conversions.h"

#include "cv_bridge/cv_bridge.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"
#include "all_seaing_interfaces/msg/labeled_bounding_box2_d.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud_array.hpp"
#include "all_seaing_interfaces/msg/labeled_object_point_cloud.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class BBoxProjectPCloud : public rclcpp::Node{
private:
    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>::SharedPtr m_object_pcl_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_object_pcl_viz_pub;
    rclcpp::Publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>::SharedPtr m_refined_object_pcl_contour_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_refined_object_pcl_viz_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_refined_object_contour_viz_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> m_bbox_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_pc_cam_tf_ok;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    // Pointcloud-camera sync policies
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::PointCloud2,
                                                            all_seaing_interfaces::msg::LabeledBoundingBox2DArray>
        PointCloudCamBBoxPolicy;
    typedef message_filters::Synchronizer<PointCloudCamBBoxPolicy> PointCloudCamBBoxSync;
    std::shared_ptr<PointCloudCamBBoxSync> m_pc_cam_bbox_sync;

    // Get intrinsic camera model information needed for projection
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame);

    // Get the point cloud from the LiDAR, the image from the camera (only to get the camera frame),
    // and the bounding boxes of the objects detected from the YOLOv8 (or other object detector/color segmentator) node,
    // and output a set of point clouds representing the points included in the bounding box of each detected object
    // (in 3D space, by projecting them onto the image and seeing if they are in the bounding box)
    void bb_pcl_project(
        const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg);

    // The HSV-similarity condition for Conditional Euclidean Clustering
    // bool hsv_diff_condition(std::vector<double> weights, double thres, const pcl::PointXYZHSV& p1, const pcl::PointXYZHSV& p2, float sq_dist);

    // The penalty function that compares an HSV color value to an HSV color range
    // double color_range_penalty(std::vector<double> weights, std::vector<int> color_range, cv::Vec3b pt_color);

    int m_obstacle_id;

    // for cluster extraction
    int m_obstacle_size_min;
    int m_obstacle_size_max;
    double m_clustering_distance;
    double m_obstacle_seg_thresh;
    double m_obstacle_drop_thresh;
    double m_polygon_area_thresh;

    // for color segmentation
    std::string color_ranges_file;
    std::string color_label_mappings_file;
    YAML::Node label_config_yaml, ranges_config_yaml;
    std::map<int, std::string> label_color_map;
    std::map<std::string, std::vector<int>> color_range_map;

    // for cluster-contour matching/selection
    std::string matching_weights_file;
    std::string contour_matching_color_ranges_file;

    YAML::Node matching_weights_config_yaml, contour_matching_ranges_config_yaml;

    double m_clustering_distance_weight;
    std::vector<double> m_clustering_color_weights;
    double m_clustering_color_thres;
    double m_cluster_contour_distance_weight;
    std::vector<double> m_cluster_contour_color_weights;
    std::vector<double> m_contour_detection_color_weights;
    double m_cluster_contour_size_weight;
    std::map<std::string, std::vector<int>> contour_matching_color_range_map;

public:
    BBoxProjectPCloud();
    virtual ~BBoxProjectPCloud();
};

#endif // ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP