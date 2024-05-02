#ifndef PROJECT_CLUSTER_BBOX_OVERLAY_HPP_
#define PROJECT_CLUSTER_BBOX_OVERLAY_HPP_

#include <all_seaing_interfaces/msg/detail/cloud_cluster_array__struct.hpp>
#include <string>
#include <vector>
#include <unordered_set>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "all_seaing_interfaces/msg/cloud_cluster_array.hpp"
#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class ClusterBboxOverlay : public rclcpp::Node
{
private:
    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::CloudClusterArray>::SharedPtr cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr text_marker_array_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr image_intrinsics_sub_;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> bbox_sub_;
    message_filters::Subscriber<all_seaing_interfaces::msg::CloudClusterArray> cluster_sub_;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped pc_cam_tf_;
    bool viz_;
    bool pc_cam_tf_ok_;

    // IntrinsicsCb camera model variables
    image_geometry::PinholeCameraModel cam_model_;

    // Cluster-bbox sync policies
    typedef message_filters::sync_policies::ApproximateTime<
        all_seaing_interfaces::msg::LabeledBoundingBox2DArray, all_seaing_interfaces::msg::CloudClusterArray>
        Cluster_Bbox_Policy;
    typedef message_filters::Synchronizer<Cluster_Bbox_Policy> Cluster_Bbox_Sync;
    std::shared_ptr<Cluster_Bbox_Sync> cluster_bbox_sync_;

    /**
     * Fuse bounding box and cluster data by projecting 3D points onto 2D image.
     * @param in_bbox_msg    shared pointer to the bbox to fuse
     * @param in_cluster_msg shared pointer to the cluster to project
     */
    void ClusterBboxFusionCb(
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
        const all_seaing_interfaces::msg::CloudClusterArray::ConstSharedPtr &in_cluster_msg);

    /**
     * Get intrinsic camera model information needed for projection
     * @param info_msg CameraInfo message that contains intrinsic camera model
     * information
     */
    void IntrinsicsCb(const sensor_msgs::msg::CameraInfo &info_msg);

    /**
     * Get transform from source frame to target frame
     * @param in_target_frame name of the target frame to transform to
     * @param in_src_frame    name of the source frame to transform from
     */
    geometry_msgs::msg::TransformStamped
    GetTransform(const std::string &in_target_frame,
                 const std::string &in_src_frame);

    void markers(const all_seaing_interfaces::msg::CloudClusterArray &in_cluster_array);

public:
    ClusterBboxOverlay();
    virtual ~ClusterBboxOverlay();
};

#endif // PROJECT_POINTCLOUD_IMAGE_OVERLAY_HPP_
