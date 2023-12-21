/*****************************************************                                                  
Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab                                                   
NAME: Michael DeFilippo                                                                             
ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA                                             
FILE: pointcloud_image_overlay.h
DATE: 2022                                                                                          
NOTE: Node to project pointcloud data into the given image space                                    

This is unreleased BETA code. no permission is granted or                                              
implied to use, copy, modify, and distribute this software                                             
except by the author(s), or those designated by the author.                                            
 ************************************************************************/

#ifndef PROJECT_POINTCLOUD_IMAGE_OVERLAY_H
#define PROJECT_POINTCLOUD_IMAGE_OVERLAY_H

#define __APP_NAME__ "pointcloud_image_overlay"

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>  // Sub/pub of ROS image msgs
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>  // change from ros img msg to CV Mat
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class PclImageOverlay
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber image_intrinsics_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    tf::TransformListener* tf_listener_;
    tf::StampedTransform pc_cam_tf_;
    bool pc_cam_tf_ok_;
    std_msgs::Header pc_sensor_header_;


    // IntrinsicsCb vars
    image_geometry::PinholeCameraModel cam_model_;
    cv::Size image_size_;
    std::vector<cv::Point> clicked_points_;

    // Sync'ed msgs cb
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::PointCloud2> PC_Cam_Policy;
    typedef message_filters::Synchronizer<PC_Cam_Policy> PC_Cam_Sync;
    boost::shared_ptr<PC_Cam_Sync> pc_cam_sync_;

    void PcImageFusionCb(const sensor_msgs::ImageConstPtr& in_img_msg,
            const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);
    /**
     * @brief callback to combine pointcloud returns with image msg
     * @param[in] in_img_msg
     * @param[in] in_cloud_msg
     **/

    void InitRosIO(ros::NodeHandle &in_private_nh);
    /**
     * @brief callback to init params and subscribtions
     * @param[in] in_private_nh
     */

    void IntrinsicsCb(const sensor_msgs::CameraInfoConstPtr& info_msg);

    tf::StampedTransform GetTransform(const std::string& in_target_frame,
            const std::string& in_src_frame);

    pcl::PointXYZ TfPoint(const pcl::PointXYZ &in_point,
            const tf::StampedTransform &in_transform);

    public:
    void Run();
    PclImageOverlay() : it_(nh_){};
    virtual ~PclImageOverlay();

};
#endif //PROJECT_POINTCLOUD_IMAGE_OVERLAY_H
