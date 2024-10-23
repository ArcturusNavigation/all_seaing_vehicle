#include "all_seaing_perception/bbox_project_pcloud.hpp"

BBoxProjectPCloud::BBoxProjectPCloud() : Node("bbox_project_pcloud"){
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    this->declare_parameter("bbox_object_margin", 0.0);

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 10, std::bind(&BBoxProjectPCloud::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "image", rmw_qos_profile_sensor_data);
    m_cloud_sub.subscribe(this, "point_cloud", rmw_qos_profile_sensor_data);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_sensor_data);
    
    // Send pc msg and img msg to bb_pcl_project
    m_pc_cam_bbox_sync =
        std::make_shared<PointCloudCamBBoxSync>(PointCloudCamBBoxPolicy(10), m_image_sub, m_cloud_sub, m_bbox_sub);
    m_pc_cam_bbox_sync->registerCallback(std::bind(&BBoxProjectPCloud::bb_pcl_project, this,
                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Publishers
    m_object_pcl_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>("labeled_object_point_clouds", 5);
}

void BBoxProjectPCloud::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

void BBoxProjectPCloud::bb_pcl_project(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {
    
    // LIDAR -> Camera transform (useful for projecting the camera bboxes onto the point cloud, have the origin on the camera frame)
    if (!m_pc_cam_tf_ok)
        // TODO: get the camera frame another way and don't subscribe to the image topic, we only need the bounding boxes which are another topic
        m_pc_cam_tf = get_tf(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);

    // Transform in_cloud_msg to the camera frame and convert PointCloud2 to PCL PointCloud
    sensor_msgs::msg::PointCloud2 in_cloud_tf;
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, m_pc_cam_tf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

    auto object_pcls = all_seaing_interfaces::msg::LabeledObjectPointCloudArray();

    // Just use the same pcloud to image projection, but check if it's within some binding box and assign it to that detection
    for (all_seaing_interfaces::msg::LabeledBoundingBox2D bbox : in_bbox_msg->boxes){
        auto labeled_pcl = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        pcl::PointCloud<pcl::PointXYZI>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        labeled_pcl.label = bbox.label;
        labeled_pcl.probability = bbox.probability;
        for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points) {
            // Project 3D point onto the image plane using the intrinsic matrix.
            // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
            cv::Point2d xy_rect = m_cam_model.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x));
            // Check if within bounds & in front of the boat
            if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
                (xy_rect.y < m_cam_model.cameraInfo().height) && (point_tf.x >= 0)) {          
                double bbox_margin = this->get_parameter("bbox_object_margin").as_double();
                // Check if point is in bbox
                if(xy_rect.x >= bbox.min_x-bbox_margin && xy_rect.x <= bbox.max_x+bbox_margin && xy_rect.y >= bbox.min_y-bbox_margin && xy_rect.y <= bbox.max_y+bbox_margin){
                    obj_cloud_ptr->push_back(point_tf);
                }
            }
        }
        pcl::toROSMsg(*obj_cloud_ptr, labeled_pcl.cloud);
        object_pcls.objects.push_back(labeled_pcl);
    }
    m_object_pcl_pub->publish(object_pcls);
}

geometry_msgs::msg::TransformStamped BBoxProjectPCloud::get_tf(const std::string &in_target_frame,
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