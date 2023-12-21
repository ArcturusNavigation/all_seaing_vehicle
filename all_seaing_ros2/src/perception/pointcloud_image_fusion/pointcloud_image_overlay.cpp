/*****************************************************                                                  
Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab                                                   
NAME: Michael DeFilippo                                                                             
ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA                                             
FILE: pointcloud_image_overlay.cpp                                                                  
DATE: 2022                                                                                          
NOTE: Node to project pointcloud data into the given image space                                    

This is unreleased BETA code. no permission is granted or                                              
implied to use, copy, modify, and distribute this software                                             
except by the author(s), or those designated by the author.                                            
 ************************************************************************/

#include "pointcloud_image_fusion/pointcloud_image_overlay.h"


void PclImageOverlay::PcImageFusionCb(const sensor_msgs::ImageConstPtr& in_img_msg,
        const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg)
{
    if (!pc_cam_tf_ok_)
    {
        pc_cam_tf_ = GetTransform(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);
    }

    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr blob_ptr;
    // wrap call to catch conversion errors
    try
    {
        // convert msg to CvImage to work with CV2, copy img since we will be modifying
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
        blob_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Cvt PointCloud2 msg to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud_msg, *in_cloud_ptr);
    pc_sensor_header_ = in_cloud_msg->header;

    // for each point in the cloud
    for(unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
        // Transform Point
        pcl::PointXYZ point;
        point.x = in_cloud_ptr->points[i].x;
        point.y = in_cloud_ptr->points[i].y;
        point.z = in_cloud_ptr->points[i].z;
        ROS_INFO("src point:         x: %f, y: %f, z: %f", point.x, point.y, point.z);
        pcl::PointXYZ point_tf;
        point_tf = TfPoint(point, pc_cam_tf_);
        //ROS_INFO("transformed point: x: %f, y: %f, z: %f", point_tf.x, point_tf.y, point_tf.z);

        /* Project 3D point onto the image plane using the intrinsic matrix
         * consisting of the camera parameters like focal length and optical center
         */
        float dist = sqrt(pow(point.x,2)+pow(point.y,2));
        cv::Point2d xy_point = cam_model_.project3dToPixel(cv::Point3d(point_tf.x,
                    point_tf.y,
                    point_tf.z));

        cv::Point2d xy_rect = cam_model_.rectifyPoint(xy_point);
        //ROS_INFO("projected 2d points (%f, %f)", xy_point.x, xy_point.y);
        //ROS_INFO("rectified 2d points (%f, %f)", xy_rect.x, xy_rect.y);
        /*
           if ((xy_point.x >=0) && (xy_point.x < image_size_.width)
           && (xy_point.y >=0) && (xy_point.y < image_size_.height))
           {
           cv::circle(cv_ptr->image, cv::Point(xy_point.x, xy_point.y),
           2, cv::Scalar(0,255,0), 5);
           }
           */
        // test code for using rectifyied point above
        if ((xy_rect.x >=0) && (xy_rect.x < image_size_.width)
                && (xy_rect.y >=0) && (xy_rect.y < image_size_.height))
        {
            cv::circle(cv_ptr->image, cv::Point(xy_rect.y, xy_rect.x),
                    2, cv::Scalar(0,255,0), 5);
        } 
    }

    if (clicked_points_.size() > 0)
        for (auto it = clicked_points_.begin(); it != clicked_points_.end(); ++it)
        {
            //ROS_INFO("Points size: %i", clicked_points_.size());
            int index = std::distance(clicked_points_.begin(), it);
            ROS_INFO("points[%i]: (%i, %i)", index, (*it).x, (*it).y);
        }      
    /* Blob detector
       cv::SimpleBlobDetector::Params params;
       params.minThreshold = 10; // 10
       params.maxThreshold = 200; // 200
       params.filterByArea = true;
       params.minArea = 10; // 10
                            //params.filterByInertia = true;
                            //params.minInertiaRatio = 0.01;
                            //params.filterByCircularity = true;
                            //params.minCircularity = 0.7;
                            params.filterByColor = true;
                            params.blobColor = 255;

                            cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

                            std::vector<cv::KeyPoint> keypoints;
                            detector->detect(blob_ptr->image, keypoints);
                            cv::Mat im;
                            cv::drawKeypoints(blob_ptr->image, keypoints, im, cv::Scalar(0,0,255),
                            cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
                            cv::imshow("image", im);
                            */
    cv::imshow("image", cv_ptr->image);
    //cam_model_.rectifyImage(cv_ptr->image, cv_ptr->image);
    // Pub result
    // convert CvImage to ROS image msg and publish
    image_pub_.publish(cv_ptr->toImageMsg());
}

pcl::PointXYZ PclImageOverlay::TfPoint(const pcl::PointXYZ &in_point,
        const tf::StampedTransform &in_transform)
{
    /* @brief 3D point transformation from world coordinates to camera coordinates                                                           
     * via Extrinsic matrix which consists of a rotation and translation                                                                     
     * between the two coordinate systems                                                                                                    
     */

    // Rotate and translate by directly multiplying point by tf transform
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

tf::StampedTransform PclImageOverlay::GetTransform(const std::string& in_target_frame,
        const std::string& in_src_frame)
{
    tf::StampedTransform tf;
    pc_cam_tf_ok_ = false;
    try
    {
        tf_listener_->lookupTransform(in_target_frame, in_src_frame, ros::Time(0), tf);
        pc_cam_tf_ok_ = true;
        ROS_INFO("[%s] Pointcloud to Camera Transform good", __APP_NAME__);
        ROS_INFO("[%s] in_target_frame: %s, in_src_frame: %s", __APP_NAME__, in_target_frame.c_str(), in_src_frame.c_str());
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
    }
    return tf;
}

void PclImageOverlay::IntrinsicsCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    image_size_.height = info_msg->height;
    image_size_.width = info_msg->width;

    cam_model_.fromCameraInfo(info_msg);

    image_intrinsics_sub_.shutdown();
    ROS_INFO("[%s] Image Intrinsics set: %i, %i", __APP_NAME__,
            image_size_.height, image_size_.width);
}

static void onMouse(int event, int x, int y, int, void *param)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        ROS_INFO("clicked point %i, %i", x, y);
        cv::Point p;
        p.x = x;
        p.y = y;
        std::vector<cv::Point>* ptPtr = (std::vector<cv::Point>*)param;
        ptPtr->push_back(p);
    }
}

void PclImageOverlay::InitRosIO(ros::NodeHandle &in_private_nh)
{
    // find points in image to perform homography
    cv::namedWindow("image");
    cv::startWindowThread();
    cv::setMouseCallback("image", onMouse, (void*)&clicked_points_);

    std::string img_src, img_info_src, cloud_src;
    in_private_nh.getParam("img_src", img_src);
    ROS_INFO("[%s] img_src: %s", __APP_NAME__, img_src.c_str());
    in_private_nh.getParam("img_info_src", img_info_src);
    ROS_INFO("[%s] img_info_src %s", __APP_NAME__, img_info_src.c_str());
    in_private_nh.getParam("cloud_src", cloud_src);
    ROS_INFO("[%s] cloud_src %s", __APP_NAME__, cloud_src.c_str());

    // Subscriptions
    image_intrinsics_sub_ = in_private_nh.subscribe(img_info_src, 1,
            &PclImageOverlay::IntrinsicsCb, this);
    ROS_INFO("[%s] cloud_sub_ subscribing to %s", __APP_NAME__, cloud_src.c_str());
    cloud_sub_.subscribe(in_private_nh, cloud_src.c_str(), 1);
    ROS_INFO("[%s] image_sub_ subscribing to %s", __APP_NAME__, img_src.c_str());
    image_sub_.subscribe(it_, img_src.c_str(), 1);

    // Send pc msg and img msg to cb
    ROS_INFO("[%s] Approximate time sync for %s and %s", __APP_NAME__,
            img_src.c_str(), cloud_src.c_str());
    pc_cam_sync_.reset(new PC_Cam_Sync(PC_Cam_Policy(10), image_sub_, cloud_sub_));
    pc_cam_sync_->registerCallback(boost::bind(&PclImageOverlay::PcImageFusionCb, this, _1, _2));

    // Publishers
    image_pub_ = it_.advertise(img_src + "/pc_image_fusion", 1);
}

void PclImageOverlay::Run()
{
    ros::NodeHandle private_nh("~");
    tf::TransformListener tf_listener;
    tf_listener_ = &tf_listener;

    InitRosIO(private_nh);

    ros::spin();
}

PclImageOverlay::~PclImageOverlay()
{
    // Destructor
}
