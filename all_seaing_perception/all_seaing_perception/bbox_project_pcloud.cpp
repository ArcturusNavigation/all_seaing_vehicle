#include "all_seaing_perception/bbox_project_pcloud.hpp"

BBoxProjectPCloud::BBoxProjectPCloud() : Node("bbox_project_pcloud"){
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&PclImageOverlay::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "image", rmw_qos_profile_sensor_data);
    m_cloud_sub.subscribe(this, "point_cloud", rmw_qos_profile_sensor_data);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_sensor_data)
    
    // Send pc msg and img msg to bb_pcl_project
    m_pc_cam_bbox_sync =
        std::make_shared<PointCloudCamSync>(PointCloudCamPolicy(10), m_image_sub, m_cloud_sub, m_bbox_sub);
    m_pc_cam_bbox_sync->registerCallback(std::bind(&PclImageOverlay::bb_pcl_project, this,
                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void BBoxProjectPCloud::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

void BBoxProjectPCloud::bb_pcl_project(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray &in_bbox_msg) {
    
    // LIDAR -> Camera transform (useful for projecting the camera bboxes onto the point cloud, have the origin on the camera frame)
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

    //TODO: Take each bounding box and filter the points of the point cloud that are "in" this bbox in 3D, using rays or smth

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

BBoxProjectPCloud::~BBoxProjectPCloud() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBoxProjectPCloud>());
    rclcpp::shutdown();
    return 0;
}