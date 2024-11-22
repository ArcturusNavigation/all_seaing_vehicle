#include "all_seaing_perception/bbox_project_pcloud.hpp"

BBoxProjectPCloud::BBoxProjectPCloud() : Node("bbox_project_pcloud"){
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    //essential ones
    this->declare_parameter("bbox_object_margin", 0.0);
    this->declare_parameter("lidar_topic", "/wamv/sensors/lidars/lidar_wamv_sensor/points");
    this->declare_parameter("camera_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw");
    this->declare_parameter("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info");

    // for cluster extraction
    this->declare_parameter<int>("obstacle_size_min", 20);
    this->declare_parameter<int>("obstacle_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("polygon_area_thresh", 100000.0);
    
    m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
    m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
    m_clustering_distance = this->get_parameter("clustering_distance").as_double();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_polygon_area_thresh = this->get_parameter("polygon_area_thresh").as_double();

    // for color segmentation
    this->declare_parameter('color_ranges_file', '');
    this->declare_parameter('color_label_mappings_file', '');

    color_ranges_file = this->get_parameter('color_ranges_file').as_string();
    color_label_mappings_file = this->get_parameter('color_label_mappings_file').as_string();

    // for cluster-contour matching & selection
    this->declare_parameter('matching_weights_file', '');
    matching_weights_file = this->get_parameter('matching_weights_file').as_string();

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        this->get_parameter("camera_info_topic").as_string(), 10, std::bind(&BBoxProjectPCloud::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, this->get_parameter("camera_topic").as_string(), rmw_qos_profile_sensor_data);
    m_cloud_sub.subscribe(this, this->get_parameter("lidar_topic").as_string(), rmw_qos_profile_sensor_data);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_sensor_data);
    
    // Send pc msg and img msg to bb_pcl_project
    m_pc_cam_bbox_sync =
        std::make_shared<PointCloudCamBBoxSync>(PointCloudCamBBoxPolicy(10), m_image_sub, m_cloud_sub, m_bbox_sub);
    m_pc_cam_bbox_sync->registerCallback(std::bind(&BBoxProjectPCloud::bb_pcl_project, this,
                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Publishers
    m_object_pcl_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>("labeled_object_point_clouds", 5);
    m_object_pcl_viz_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_point_clouds_viz", 5);

    // get color label mappings from yaml
    std::ifstream label_yaml(color_label_mappings_file);
    if (label_yaml.is_open()) {
        label_config_yaml = YAML::Load(label_yaml);  
        for (YAML::const_iterator it = label_config_yaml.begin(); it != label_config_yaml.end(); ++it) {
            // RCLCPP_INFO(this->get_logger(), "label %d and color %s", it->second.as<int>(), it->second.as<std::string>().c_str());
            label_color_map[it->second.as<int>()] = it->first.as<std::string>();
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", color_label_mappings_file.c_str());
    }
    label_yaml.close();

    // get color ranges from yaml
    std::ifstream ranges_yaml(color_ranges_file);
    if (ranges_yaml.is_open()) {
        ranges_config_yaml = YAML::Load(ranges_yaml);  
        for (YAML::const_iterator it = ranges_config_yaml.begin(); it != ranges_config_yaml.end(); ++it) {
            // RCLCPP_INFO(this->get_logger(), "label %d and color %s", it->second.as<int>(), it->second.as<std::string>().c_str());
            color_range_map[it->first.as<std::string>()] = it->second.as<int[6]>();
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", color_ranges_file.c_str());
    }
    
    ranges_yaml.close();

    // get cluster-contour matching & selection parameters from yaml
    std::ifstream matching_yaml(matching_weights_file);
    if (matching_yaml.is_open()) {
        matching_weights_yaml = YAML::Load(matching_yaml);

        m_clustering_distance_weight = matching_weights_yaml["clustering_distance_weight"];
        m_clustering_color_weights = matching_weights_yaml["clustering_color_weights"];
        m_clustering_color_thres = matching_weights_yaml["clustering_color_thres"];
        m_cluster_contour_distance_weight = matching_weights_yaml["cluster_contour_distance_weight"];
        m_cluster_contour_color_weights = matching_weights_yaml["cluster_contour_color_weights"];
        m_contour_detection_color_weights = matching_weights_yaml["contour_detection_color_weights"];
        m_cluster_contour_size_weight = matching_weights_yaml["cluster_contour_size_weight"];
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", matching_weights_file.c_str());
    }
}

void BBoxProjectPCloud::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    RCLCPP_INFO(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

bool hsv_diff_condition(const pcl::PointXYZHSV& p1, const pcl::PointXYZHSV& p2, float sq_dist){
    return m_clustering_color_weights[0]*p1.h^2+m_clustering_color_weights[1]*p1.s^2+m_clustering_color_weights[2]*p1.v^2<m_clustering_color_thres^2;
}

void BBoxProjectPCloud::bb_pcl_project(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {
    RCLCPP_INFO(this->get_logger(), "GOT DATA");
    // LIDAR -> Camera transform (useful for projecting the camera bboxes onto the point cloud, have the origin on the camera frame)
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);

    // Transform in_cloud_msg to the camera frame and convert PointCloud2 to PCL PointCloud
    sensor_msgs::msg::PointCloud2 in_cloud_tf;
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, m_pc_cam_tf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

    RCLCPP_INFO(this->get_logger(), "%d POINTS, %d OBJECTS", in_cloud_tf_ptr->points.size(), in_bbox_msg->boxes.size());
    RCLCPP_INFO(this->get_logger(), "FRAME_ID BEFORE SELECTION, AFTER TRANSFORM TO CAMERA FRAME: %s", in_cloud_tf_ptr->header.frame_id);

    auto object_pcls = all_seaing_interfaces::msg::LabeledObjectPointCloudArray();
    std::vector<pcl::PointCloud<pcl::PointXYZI>> obj_cloud_vec;
    int max_len = 0;
    // Just use the same pcloud to image projection, but check if it's within some binding box and assign it to that detection
    int obj = 0;
    for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points) {
        // Project 3D point onto the image plane using the intrinsic matrix.
        // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
        RCLCPP_DEBUG(this->get_logger(), "3D POINT: (%lf, %lf, %lf)", point_tf.x, point_tf.y, point_tf.z);
        cv::Point2d xy_rect = m_cam_model.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x));
        // Check if within bounds & in front of the boat
        RCLCPP_DEBUG(this->get_logger(), "POINT PROJECTED ONTO IMAGE: (%lf, %lf)", xy_rect.x, xy_rect.y);
    }
    // Convert msg to CvImage to work with CV2. Copy img since we will be modifying.
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImagePtr cv_hsv_ptr = cv_bridge::cvtColor(cv_ptr.toCvShare(), cv::COLOR_RGB2HSV);
    std::vector<std::pair<all_seaing_interfaces::msg::LabeledBoundingBox2D, pcl::PointCloud<pcl::PointXYZHSV>>::Ptr> bbox_pcloud_objects;
    for (all_seaing_interfaces::msg::LabeledBoundingBox2D bbox : in_bbox_msg->boxes){
        auto labeled_pcl = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        labeled_pcl.label = bbox.label;
        obj_cloud_ptr->header = in_cloud_tf_ptr->header;
        RCLCPP_DEBUG(this->get_logger(), "BOUNDING BOX FOR OBJECT %d: (%lf,%lf), (%lf, %lf)", obj, bbox.min_x, bbox.min_y, bbox.max_x, bbox.max_y, obj);
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
                    cv::Vec3b hsv = cv_hsv_ptr->image.at<cv::Vec3b>(xy_rect);
                    // cv::Vec3b rgb = cv_ptr->image.at<cv::Vec3b>(xy_rect);
                    obj_cloud_ptr->push_back(pcl::PointXYZHSV(point_tf.x, point_tf.y, point_tf.z, hsv[0], hsv[1], hsv[2]));
                    // obj_cloud_ptr->push_back(pcl::PointXYZRGB(point_tf.x, point_tf.y, point_tf.z, rgb[0], rgb[1], rgb[2]));
                    RCLCPP_DEBUG(this->get_logger(), "SELECTED HSV POINT PROJECTED ONTO IMAGE: (%lf, %lf) -> (%lf, %lf, %lf)", xy_rect.x, xy_rect.y, hsv[0], hsv[1], hsv[2]);
                    // RCLCPP_DEBUG(this->get_logger(), "SELECTED RGB POINT PROJECTED ONTO IMAGE: (%lf, %lf) -> (%lf, %lf, %lf)", xy_rect.x, xy_rect.y, rgb[0], rgb[1], rgb[2]);
                }
            }
        }
        pcl::toROSMsg(*obj_cloud_ptr, labeled_pcl.cloud);
        object_pcls.objects.push_back(labeled_pcl);
        obj_cloud_vec.push_back(*obj_cloud_ptr);
        bbox_pcloud_objects.push_back(std::make_pair(bbox, obj_cloud_ptr));
        obj++;
        RCLCPP_INFO(this->get_logger(), "%d POINTS IN OBJECT %d", obj_cloud_ptr->size(), obj);
        max_len = std::max(max_len, (int)obj_cloud_ptr->size());
    }
    RCLCPP_INFO(this->get_logger(), "WILL NOW SEND OBJECT POINT CLOUDS");
    m_object_pcl_pub->publish(object_pcls);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_obj_pcls_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    all_obj_pcls_ptr->header = in_cloud_tf_ptr->header;
    //convert vector of PointCloud to a single PointCloud with channels
    all_obj_pcls_ptr->resize((pcl::uindex_t)max_len, (pcl::uindex_t)in_bbox_msg->boxes.size());
    RCLCPP_INFO(this->get_logger(), "PUBLISHED PCLOUD DIMENSIONS: height: %d, width: %d", (int)all_obj_pcls_ptr->height, (int)all_obj_pcls_ptr->width);
    RCLCPP_INFO(this->get_logger(), "STORED PCLOUD DIMENSIONS: objects: %d, max_length: %d", (int)obj_cloud_vec.size(), max_len);
    try{
        //TODO: CHECK IF THE CHANNELS CORRECTLY REPRESENT DIFFERENT OBJECTS
        //(I.E. HEIGHT & WIDTH ARE CORRECT, OTHERWISE SWAP THEIR ORDER BOTH IN THE RESIZE() AND THE AT() METHODS)
        for(int i = 0; i<obj_cloud_vec.size(); i++){
            for(int j = 0; j<obj_cloud_vec[i].size(); j++){
                all_obj_pcls_ptr->at(j,i) = obj_cloud_vec[i][j];
            }
        }
    }catch(std::exception &ex){
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    auto obj_pcls_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*all_obj_pcls_ptr, obj_pcls_msg);
    m_object_pcl_viz_pub->publish(obj_pcls_msg);

    // REFINE OBJECT POINT CLOUDS
    for(auto bbox_pcloud_pair : bbox_pcloud_objects){
        all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = bbox_pcloud_pair.first;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr = bbox_pcloud_pair.second;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud_ptr = bbox_pcloud_pair.second;
        // pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr_hsv(new pcl::PointCloud<pcl::PointXYZHSV>);
        // // CONVERT RGB POINTS TO HSV
        // for(auto pt : pcloud_ptr->points){
        //     pcl::PointXYZHSV pt_hsv;
        //     pcl::PointXYZRGBtoXYZHSV(*pt, pt_hsv);
        //     pcloud_ptr_hsv->push_back(pt_hsv);
        // }

        //extract clusters
        pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>());
        // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
        std::vector<pcl::PointIndices> obstacles_indices;

        // EUCLIDEAN CLUSTERING (DEPRECATED)
        // // pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        // ec.setClusterTolerance(m_clustering_distance);
        // ec.setMinClusterSize(m_obstacle_size_min);
        // ec.setMaxClusterSize(m_obstacle_size_max);
        // ec.setSearchMethod(tree);
        // ec.setInputCloud(pcloud_ptr);
        // ec.extract(obstacles_indices);

        // CONDITIONAL (WITH HSV-BASED CONDITION) EUCLIDEAN CLUSTERING
        pcl::ConditionalEuclideanClustering<pcl::PointHSV> cec;
        cec.setClusterTolerance(m_clustering_distance);
        cec.setMinClusterSize(m_obstacle_size_min);
        cec.setMaxClusterSize(m_obstacle_size_max);
        cec.setSearchMethod(tree);
        cec.setInputCloud(pcloud_ptr);
        cec.setConditionFunction(&hsv_diff_condition);
        cec.segment(obstacles_indices);

        // color segmentation using the color label
        cv::Mat hsv_img(cv_hsv_ptr->image, cv::Range(bbox.min_x, bbox.max_x), cv::Range(bbox.min_y, bbox.max_y));
        cv::Size img_sz;
        cv::Point bbox_offset;
        hsv_img.locateROI(img_sz, bbox_offset);
        int lims[6] = label_color_map[bbox.label]=="red"?color_range_map["red2"]:color_range_map[label_color_map[bbox.label]];
        int h_min=lims[0], h_max=lims[1], s_min=lims[2], s_max=lims[3], v_min=lims[4], v_max=lims[5];
        cv::Mat mask;
        cv::inRange(hsv_img, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, 5));
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, 7));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // Convert the contour points to fit the original image (using image_sz and bbox_offset -> just bbox_offset+pt_coords) to be able to use it with the LiDAR (projected) points
        for(int ctr = 0; ctr < contours.size(); ctr++){
            for(int pt=0; pt < contours[ctr].size(); pt++){
                contours[ctr][pt]+=bbox_offset;
            }
        }
        //TODO: GO THROUGH ALL THE CLUSTER/CONTOUR PAIRS AND FIND THE BEST ONE BASED ON THE OPTIMALITY METRIC WITH THE WEIGHTS
        for (std::vector<cv::Point> contour : contours){
            for (pcl::PointIndices obstacle : obstacles_indices) {

            }
        }
    }
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