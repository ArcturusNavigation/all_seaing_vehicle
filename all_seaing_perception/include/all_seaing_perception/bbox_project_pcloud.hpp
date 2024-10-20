#ifndef ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP
#define ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP

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
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class BBoxProjectPointCloud : public rclcpp::Node{
private:
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_image_intrinsics_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> m_image_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;

    // Transform variables
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    geometry_msgs::msg::TransformStamped m_pc_cam_tf;
    bool m_pc_cam_tf_ok;

    // Intrinsics callback camera model variables
    image_geometry::PinholeCameraModel m_cam_model;

    // Pointcloud-camera sync policies
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                            sensor_msgs::msg::PointCloud2>
        PointCloudCamPolicy;
    typedef message_filters::Synchronizer<PointCloudCamPolicy> PointCloudCamSync;
    std::shared_ptr<PointCloudCamSync> m_pc_cam_sync;
public:
    BBoxProjectPointCloud();
    virtual ~BBoxProjectPointCloud();
};

#endif // ALL_SEAING_PERCEPTION__BBOX_PROJECT_PCLOUD_HPP