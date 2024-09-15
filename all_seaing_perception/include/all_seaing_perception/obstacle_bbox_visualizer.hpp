#ifndef ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_VISUALIZER_HPP
#define ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_VISUALIZER_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "image_geometry/pinhole_camera_model.h"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "all_seaing_interfaces/msg/obstacle_map.hpp"
#include "all_seaing_interfaces/msg/labeled_bounding_box2_d_array.hpp"
#include <opencv2/opencv.hpp>

class ObstacleBboxVisualizer : public rclcpp::Node {
private:
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_image_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::ObstacleMap> m_obstacle_map_sub;
    message_filters::Subscriber<all_seaing_interfaces::msg::LabeledBoundingBox2DArray> m_bbox_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_pc_cam_tf_ok;

    // Camera model
    image_geometry::PinholeCameraModel m_cam_model;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        all_seaing_interfaces::msg::ObstacleMap,
        all_seaing_interfaces::msg::LabeledBoundingBox2DArray>
        ObstacleBboxVisualizerPolicy;
    typedef message_filters::Synchronizer<ObstacleBboxVisualizerPolicy> ObstacleBboxVisualizerSync;
    std::shared_ptr<ObstacleBboxVisualizerSync> m_obstacle_bbox_visualizer_sync;

    // Callback for synchronized image and obstacle map
    void image_obstacle_cb(
        const sensor_msgs::msg::Image::ConstSharedPtr& in_img_msg,
        const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr& in_map_msg,
        const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr& in_bbox_msg);

    // Callback for camera info
    void intrinsics_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);

    // Helper function to get color for label
    cv::Scalar get_color_for_label(const int& label);

    geometry_msgs::msg::TransformStamped get_tf(const std::string &in_target_frame,
                                                 const std::string &in_src_frame);

public:
    ObstacleBboxVisualizer();
    virtual ~ObstacleBboxVisualizer();
};

#endif // ALL_SEAING_PERCEPTION__OBSTACLE_BBOX_VISUALIZER_HPP