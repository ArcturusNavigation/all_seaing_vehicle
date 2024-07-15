#include "all_seaing_perception/point_cloud_image_overlay.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "cv_bridge/cv_bridge.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

void PclImageOverlay::pc_image_fusion_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg) {

    // Get transform the first iteration
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);

    // Convert msg to CvImage to work with CV2. Copy img since we will be modifying.
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Transform in_cloud_msg and convert PointCloud2 to PCL PointCloud
    sensor_msgs::msg::PointCloud2 in_cloud_tf;
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, m_pc_cam_tf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

    for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points) {

        // Project 3D point onto the image plane using the intrinsic matrix.
        // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
        cv::Point2d xy_rect =
            m_cam_model.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x));

        // Plot projected point onto image if within bounds and in front of the boat
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
            (xy_rect.y < m_cam_model.cameraInfo().height) && (point_tf.x >= 0)) {
            cv::circle(cv_ptr->image, cv::Point(xy_rect.x, xy_rect.y), 2, cv::Scalar(255, 0, 0), 4);
        }
    }
    m_image_pub->publish(*cv_ptr->toImageMsg());
}

geometry_msgs::msg::TransformStamped PclImageOverlay::get_tf(const std::string &in_target_frame,
                                                             const std::string &in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    m_pc_cam_tf_ok = false;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        m_pc_cam_tf_ok = true;
        RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good");
        RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s",
                    in_target_frame.c_str(), in_src_frame.c_str());
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return tf;
}

void PclImageOverlay::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

PclImageOverlay::PclImageOverlay() : Node("point_cloud_image_overlay") {
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&PclImageOverlay::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "image", rmw_qos_profile_sensor_data);
    m_cloud_sub.subscribe(this, "point_cloud", rmw_qos_profile_sensor_data);

    // Send pc msg and img msg to pc_image_fusion_cb
    m_pc_cam_sync =
        std::make_shared<PointCloudCamSync>(PointCloudCamPolicy(10), m_image_sub, m_cloud_sub);
    m_pc_cam_sync->registerCallback(std::bind(&PclImageOverlay::pc_image_fusion_cb, this,
                                              std::placeholders::_1, std::placeholders::_2));

    // Publishers
    m_image_pub = this->create_publisher<sensor_msgs::msg::Image>("image/overlaid", 5);
}

PclImageOverlay::~PclImageOverlay() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PclImageOverlay>());
    rclcpp::shutdown();
    return 0;
}
