#include "all_seaing_vehicle/pointcloud_image_overlay.hpp"

void PclImageOverlay::PcImageFusionCb(const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
									  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg)
{
	// Get transform the first iteration
	if (!pc_cam_tf_ok_)
		pc_cam_tf_ = GetTransform(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);

	// Convert msg to CvImage to work with CV2. Copy img since we will be modifying.
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception &e)
	{
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	// Transform in_cloud_msg and convert PointCloud2 to PCL PointCloud
	sensor_msgs::msg::PointCloud2 in_cloud_tf;
	tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, pc_cam_tf_);
	pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

	for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points)
	{
		// Project 3D point onto the image plane using the intrinsic matrix.
		// Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
		cv::Point2d xy_rect = cam_model_.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x));

		// Plot projected point onto image if within bounds and in front of the boat
		if ((xy_rect.x >= 0) && (xy_rect.x < cam_model_.cameraInfo().width) &&
			(xy_rect.y >= 0) && (xy_rect.y < cam_model_.cameraInfo().height) && (point_tf.x >= 0))
		{
			cv::circle(cv_ptr->image, cv::Point(xy_rect.x, xy_rect.y), 2, cv::Scalar(255, 0, 0), 4);
		}

		//        RCLCPP_INFO(this->get_logger(), "Transformed 3D point:   x: %f, y: %f, z: %f", point_tf.x, point_tf.y, point_tf.z);
		//        RCLCPP_INFO(this->get_logger(), "Projected 2D point:     x: %f, y: %f", xy_rect.x, xy_rect.y);
	}
	image_pub_->publish(*cv_ptr->toImageMsg());
}

geometry_msgs::msg::TransformStamped PclImageOverlay::GetTransform(const std::string &in_target_frame,
																   const std::string &in_src_frame)
{
	geometry_msgs::msg::TransformStamped tf;
	pc_cam_tf_ok_ = false;
	try
	{
		tf = tf_buffer_->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
		pc_cam_tf_ok_ = true;
		RCLCPP_INFO(this->get_logger(), "LiDAR to Camera Transform good");
		RCLCPP_INFO(this->get_logger(), "in_target_frame: %s, in_src_frame: %s", in_target_frame.c_str(), in_src_frame.c_str());
	}
	catch (tf2::TransformException &ex)
	{
		RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
	}
	return tf;
}

void PclImageOverlay::IntrinsicsCb(const sensor_msgs::msg::CameraInfo &info_msg)
{
	cam_model_.fromCameraInfo(info_msg);
	//    RCLCPP_INFO(this->get_logger(), "Image Intrinsics set: %i, %i", cam_model_.cameraInfo().width, cam_model_.cameraInfo().height);
}

PclImageOverlay::PclImageOverlay() : Node("pointcloud_image_overlay")
{
	// Initialize tf_listener pointer
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Subscriptions
	image_intrinsics_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/img_info_src",
		1,
		std::bind(&PclImageOverlay::IntrinsicsCb, this, std::placeholders::_1));
	image_sub_.subscribe(this, "/img_src", rmw_qos_profile_system_default);
	cloud_sub_.subscribe(this, "/cloud_src", rmw_qos_profile_system_default);

	// Send pc msg and img msg to PCImageFusionCb
	pc_cam_sync_ = std::make_shared<PC_Cam_Sync>(PC_Cam_Policy(10), image_sub_, cloud_sub_);
	pc_cam_sync_->registerCallback(std::bind(&PclImageOverlay::PcImageFusionCb, this, std::placeholders::_1, std::placeholders::_2));

	// Publishers
	image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/pc_image_fusion", 1);
}

PclImageOverlay::~PclImageOverlay() {}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PclImageOverlay>());
	rclcpp::shutdown();
	return 0;
}
