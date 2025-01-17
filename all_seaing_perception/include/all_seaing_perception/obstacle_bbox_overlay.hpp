#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_OVERLAY_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_OVERLAY_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "image_geometry/pinhole_camera_model.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"

class ObstacleBboxOverlay : public rclcpp::Node {
private:
    // Publishers and subscribers
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr m_map_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> m_bbox_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::ObstacleMap> m_map_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_pc_cam_tf_ok;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    // Obstacle-bbox sync policies
    typedef message_filters::sync_policies::ApproximateTime<
        all_seaing_interfaces::msg::LabeledBoundingBox2DArray,
        all_seaing_interfaces::msg::ObstacleMap>
        ObstacleBboxPolicy;
    typedef message_filters::Synchronizer<ObstacleBboxPolicy> ObstacleBboxSync;
    std::shared_ptr<ObstacleBboxSync> m_obstacle_bbox_sync;

    // Get the closest obstacle
    // Diff criteria options (centroid matching, IoU)
    int get_matching_obstacle_centroid(
    const all_seaing_interfaces::msg::Obstacle &obstacle,
        const std::unordered_set<int> &chosen_indices,
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg);

    int get_matching_obstacle_iou(
    const all_seaing_interfaces::msg::Obstacle &obstacle,
        const std::unordered_set<int> &chosen_indices,
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg);

    // Fuse image and obstacle by projecting 3D points onto 2D image.
    void obstacle_bbox_fusion_cb(
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg,
        const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &in_map_msg);

    // Get intrinsic camera model information needed for projection
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg);

    // Get transform from source frame to target frame
    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                const std::string &in_src_frame);

public:
    ObstacleBboxOverlay();
    virtual ~ObstacleBboxOverlay();
};

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_OVERLAY_HPP