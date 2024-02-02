/*****************************************************                                                  
Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab
NAME: Michael DeFilippo
ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA
FILE: pointcloud_image_overlay.hpp
DATE: 2022
NOTE: Node to project pointcloud data into the given image space

This is unreleased BETA code. no permission is granted or
implied to use, copy, modify, and distribute this software
except by the author(s), or those designated by the author.
 ************************************************************************/

#ifndef PROJECT_POINTCLOUD_IMAGE_OVERLAY_HPP_
#define PROJECT_POINTCLOUD_IMAGE_OVERLAY_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "image_geometry/pinhole_camera_model.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class PclImageOverlay : public rclcpp::Node {
    private:
        // Publishers and subscribers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr image_intrinsics_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;

        // Transform variables
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped pc_cam_tf_;
        bool pc_cam_tf_ok_;

        // IntrinsicsCb camera model variables
        image_geometry::PinholeCameraModel cam_model_;

        // Pointcloud-camera sync policies
        typedef message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> PC_Cam_Policy;
        typedef message_filters::Synchronizer<PC_Cam_Policy> PC_Cam_Sync;
        std::shared_ptr<PC_Cam_Sync> pc_cam_sync_;

        /**
         * Fuse image and pointcloud data by projecting 3D points onto 2D image. Publishes projected points and image to /pc_image_fusion.
         * @param in_img_msg   shared pointer to the image to fuse
         * @param in_cloud_msg shared pointer to the pointcloud to project
         */
        void PcImageFusionCb(
            const sensor_msgs::msg::Image::ConstSharedPtr & in_img_msg,
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud_msg);

        /**
         * Get intrinsic camera model information needed for projection
         * @param info_msg CameraInfo message that contains intrinsic camera model information
         */
        void IntrinsicsCb(const sensor_msgs::msg::CameraInfo & info_msg);

        /**
         * Get transform from source frame to target frame
         * @param in_target_frame name of the target frame to transform to
         * @param in_src_frame    name of the source frame to transform from
         */
        geometry_msgs::msg::TransformStamped GetTransform(
            const std::string & in_target_frame,
            const std::string & in_src_frame);

    public:
        PclImageOverlay();
        virtual ~PclImageOverlay();
};

#endif //PROJECT_POINTCLOUD_IMAGE_OVERLAY_HPP_
