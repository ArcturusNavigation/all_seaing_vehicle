#include "all_seaing_perception/bbox_project_pcloud.hpp"

BBoxProjectPCloud::BBoxProjectPCloud() : Node("bbox_project_pcloud"){
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    //essential ones
    this->declare_parameter<double>("bbox_object_margin", 0.0);
    m_bbox_margin = this->get_parameter("bbox_object_margin").as_double();

    // for cluster extraction
    this->declare_parameter<int>("obstacle_size_min", 20);
    this->declare_parameter<int>("obstacle_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    
    m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
    m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
    m_clustering_distance = this->get_parameter("clustering_distance").as_double();

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    // for color segmentation
    this->declare_parameter("color_label_mappings_file", "");

    color_label_mappings_file = this->get_parameter("color_label_mappings_file").as_string();

    // for cluster-contour matching & selection
    this->declare_parameter("matching_weights_file", "");
    this->declare_parameter("contour_matching_color_ranges_file", "");

    matching_weights_file = this->get_parameter("matching_weights_file").as_string();
    contour_matching_color_ranges_file = this->get_parameter("contour_matching_color_ranges_file").as_string();

    // Subscriptions
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_topic", 10, std::bind(&BBoxProjectPCloud::intrinsics_cb, this, std::placeholders::_1));
    m_image_sub.subscribe(this, "camera_topic", rmw_qos_profile_sensor_data);
    m_cloud_sub.subscribe(this, "lidar_topic", rmw_qos_profile_sensor_data);
    m_bbox_sub.subscribe(this, "bounding_boxes", rmw_qos_profile_sensor_data);
    
    // Send pc msg and img msg to bb_pcl_project
    m_pc_cam_bbox_sync =
        std::make_shared<PointCloudCamBBoxSync>(PointCloudCamBBoxPolicy(10), m_image_sub, m_cloud_sub, m_bbox_sub);
    m_pc_cam_bbox_sync->registerCallback(std::bind(&BBoxProjectPCloud::bb_pcl_project, this,
                                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Publishers
    m_object_pcl_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>("labeled_object_point_clouds", 5);
    m_object_pcl_viz_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_point_clouds_viz", 5);
    m_refined_object_pcl_segment_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>("refined_object_point_clouds_segments", 5);
    m_refined_object_pcl_viz_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("refined_object_point_clouds_viz", 5);
    m_refined_object_segment_viz_pub = this->create_publisher<sensor_msgs::msg::Image>("refined_object_segments_viz", 5);

    // get color label mappings from yaml
    RCLCPP_DEBUG(this->get_logger(), "READING COLOR LABEL MAPPINGS");
    std::ifstream label_yaml(color_label_mappings_file);
    if (label_yaml.is_open()) {
        label_config_yaml = YAML::Load(label_yaml);  
        for (YAML::const_iterator it = label_config_yaml.begin(); it != label_config_yaml.end(); ++it) {
            for(int label : it->second.as<std::vector<int>>()){
                label_color_map[label] = it->first.as<std::string>();
                RCLCPP_DEBUG(this->get_logger(), "%d -> %s", label, it->first.as<std::string>().c_str());
            }
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", color_label_mappings_file.c_str());
    }
    label_yaml.close();

    // get cluster-contour matching & selection parameters from yaml
    RCLCPP_DEBUG(this->get_logger(), "READING MATCHING PARAMETERS");
    std::ifstream matching_yaml(matching_weights_file);
    if (matching_yaml.is_open()) {
        matching_weights_config_yaml = YAML::Load(matching_yaml);

        m_clustering_color_weights = matching_weights_config_yaml["clustering_color_weights"].as<std::vector<double>>();
        m_clustering_color_thres = matching_weights_config_yaml["clustering_color_thres"].as<double>();
        m_cluster_contour_distance_weight = matching_weights_config_yaml["cluster_contour_distance_weight"].as<double>();
        m_cluster_contour_color_weights = matching_weights_config_yaml["cluster_contour_color_weights"].as<std::vector<double>>();
        m_contour_detection_color_weights = matching_weights_config_yaml["contour_detection_color_weights"].as<std::vector<double>>();
        m_cluster_contour_size_weight = matching_weights_config_yaml["cluster_contour_size_weight"].as<double>();
    
        RCLCPP_DEBUG(this->get_logger(), "clustering_color_weights: [%lf, %lf, %lf]", m_clustering_color_weights[0], m_clustering_color_weights[1], m_clustering_color_weights[2]);
        RCLCPP_DEBUG(this->get_logger(), "clustering_color_thres: %lf", m_clustering_color_thres);
        RCLCPP_DEBUG(this->get_logger(), "cluster_contour_distance_weight: %lf", m_cluster_contour_distance_weight);
        RCLCPP_DEBUG(this->get_logger(), "cluster_contour_color_weights: [%lf, %lf, %lf]", m_cluster_contour_color_weights[0], m_cluster_contour_color_weights[1], m_cluster_contour_color_weights[2]);
        RCLCPP_DEBUG(this->get_logger(), "contour_detection_color_weights: [%lf, %lf, %lf]", m_contour_detection_color_weights[0], m_contour_detection_color_weights[1], m_contour_detection_color_weights[2]);
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", matching_weights_file.c_str());
    }
    matching_yaml.close();

    // get contour matching color ranges from yaml
    RCLCPP_DEBUG(this->get_logger(), "READING CONTOUR MATCHING COLOR RANGES");
    std::ifstream contour_ranges_yaml(contour_matching_color_ranges_file);
    if (contour_ranges_yaml.is_open()) {
        contour_matching_ranges_config_yaml = YAML::Load(contour_ranges_yaml);  
        for (YAML::const_iterator it = contour_matching_ranges_config_yaml.begin(); it != contour_matching_ranges_config_yaml.end(); ++it) {
            contour_matching_color_range_map[it->first.as<std::string>()] = it->second.as<std::vector<int>>();
            RCLCPP_DEBUG(this->get_logger(), "%s -> [%d, %d, %d, %d, %d, %d]", it->first.as<std::string>().c_str(), it->second.as<std::vector<int>>()[0], it->second.as<std::vector<int>>()[1], it->second.as<std::vector<int>>()[2], it->second.as<std::vector<int>>()[3], it->second.as<std::vector<int>>()[4], it->second.as<std::vector<int>>()[5]);
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", contour_matching_color_ranges_file.c_str());
    }
    contour_ranges_yaml.close();
}

void BBoxProjectPCloud::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    RCLCPP_DEBUG(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

// The HSV-similarity condition for Conditional Euclidean Clustering
bool hsv_diff_condition(std::vector<double> weights, double thres, const pcl::PointXYZHSV& p1, const pcl::PointXYZHSV& p2, float sq_dist){
    return weights[0]*(p1.h-p2.h)*(p1.h-p2.h)+weights[1]*(p1.s-p2.s)*(p1.s-p2.s)+weights[2]*(p1.v-p2.v)*(p1.v-p2.v) < thres*thres;
}

double line_range_penalty(int a, int b, int x){
    // Average of squared distances from each point in the range
    return (a<=x && x<=b)?0:(pow(b-x,3)-pow(x-a,3))/(b-a);
}

// The penalty function that compares an HSV color value to an HSV color range
double color_range_penalty(std::vector<double> weights, std::vector<int> color_range, std::vector<long long> pt_color){
    double h_min=color_range[0], h_max=color_range[1], s_min=color_range[2], s_max=color_range[3], v_min=color_range[4], v_max=color_range[5];
    return weights[0]*line_range_penalty(h_min, h_max, pt_color[0]) + weights[1]*line_range_penalty(s_min, s_max, pt_color[1]) + weights[2]*line_range_penalty(v_min, v_max, pt_color[2]);
}

cv::Vec3b int_to_bgr(double i, double k){
    cv::Mat hsvimg(1,1,CV_8UC3,cv::Vec3b(i/k*180,255,255));
    cv::Mat bgrimg;
    cv::cvtColor(hsvimg, bgrimg, cv::COLOR_HSV2BGR);
    return bgrimg.at<cv::Vec3b>(0,0);
}

void BBoxProjectPCloud::bb_pcl_project(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {
    RCLCPP_DEBUG(this->get_logger(), "GOT DATA");

    // LIDAR -> Camera transform (useful for projecting the camera bboxes onto the point cloud, have the origin on the camera frame)
    if (!m_pc_cam_tf_ok)
        m_pc_cam_tf = get_tf(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id);

    // Transform in_cloud_msg to the camera frame and convert PointCloud2 to PCL PointCloud
    sensor_msgs::msg::PointCloud2 in_cloud_tf;
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, m_pc_cam_tf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

    RCLCPP_DEBUG(this->get_logger(), "%d POINTS, %d OBJECTS", in_cloud_tf_ptr->points.size(), in_bbox_msg->boxes.size());
    RCLCPP_DEBUG(this->get_logger(), "FRAME_ID BEFORE SELECTION, AFTER TRANSFORM TO CAMERA FRAME: %s", in_cloud_tf_ptr->header.frame_id);

    auto object_pcls = all_seaing_interfaces::msg::LabeledObjectPointCloudArray();
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>> obj_cloud_vec;
    // int max_len = 0;
    // Just use the same pcloud to image projection, but check if it's within some binding box and assign it to that detection
    int obj = 0;
    for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points) {
        // Project 3D point onto the image plane using the intrinsic matrix.
        // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
        RCLCPP_DEBUG(this->get_logger(), "3D POINT: (%lf, %lf, %lf)", point_tf.x, point_tf.y, point_tf.z);
        cv::Point2d xy_rect = m_is_sim? m_cam_model.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x)) : m_cam_model.project3dToPixel(cv::Point3d(point_tf.x, point_tf.y, point_tf.z));
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
    //show received image
    // cv::imshow("Received Image", cv_ptr->image);
    // cv::waitKey();
    cv::Mat cv_hsv;
    cv::cvtColor(cv_ptr->image, cv_hsv, cv::COLOR_BGR2HSV);
    //show hsv (false color) image
    // cv::imshow("HSV Image", cv_hsv);
    // cv::waitKey();
    std::vector<std::pair<all_seaing_interfaces::msg::LabeledBoundingBox2D, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>> bbox_pcloud_objects;
    for (all_seaing_interfaces::msg::LabeledBoundingBox2D bbox : in_bbox_msg->boxes){
        RCLCPP_DEBUG(this->get_logger(), "BBOX LABEL: %d", bbox.label);
        if(!label_color_map.count(bbox.label)) continue; //ignore objects that are not registered buoy types

        auto labeled_pcl = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        labeled_pcl.time = in_cloud_msg->header.stamp;
        labeled_pcl.label = bbox.label;
        obj_cloud_ptr->header = in_cloud_tf_ptr->header;
        // Add padding to bbox
        bbox.min_x -= m_bbox_margin;
        bbox.max_x += m_bbox_margin;
        bbox.min_y -= m_bbox_margin;
        bbox.max_y += m_bbox_margin;
        bbox.min_x = std::max((int)bbox.min_x,0);
        bbox.max_x = std::min((int)bbox.max_x+1, cv_hsv.cols);
        bbox.min_y = std::max((int)bbox.min_y,0);
        bbox.max_y = std::min((int)bbox.max_y+1, cv_hsv.rows);
        RCLCPP_DEBUG(this->get_logger(), "PADDED BOUNDING BOX FOR OBJECT %d: (%d,%d), (%d, %d)", obj, bbox.min_x, bbox.min_y, bbox.max_x, bbox.max_y);
        for (pcl::PointXYZI &point_tf : in_cloud_tf_ptr->points) {
            // Project 3D point onto the image plane using the intrinsic matrix.
            // Gazebo has a different coordinate system, so the y, z, and x coordinates are modified.
            cv::Point2d xy_rect = m_is_sim? m_cam_model.project3dToPixel(cv::Point3d(point_tf.y, point_tf.z, -point_tf.x)) : m_cam_model.project3dToPixel(cv::Point3d(point_tf.x, point_tf.y, point_tf.z));
            // Check if within bounds & in front of the boat
            if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
                (xy_rect.y < m_cam_model.cameraInfo().height) && (point_tf.x >= 0)) {          
                // Check if point is in bbox
                if(xy_rect.x >= bbox.min_x && xy_rect.x <= bbox.max_x && xy_rect.y >= bbox.min_y && xy_rect.y <= bbox.max_y){
                    cv::Vec3b hsv_vec3b = cv_hsv.at<cv::Vec3b>(xy_rect);
                    std::vector<long long> hsv = {(long long)hsv_vec3b[0], (long long)hsv_vec3b[1], (long long)hsv_vec3b[2]};
                    // cv::Vec3b bgr = cv_ptr->image.at<cv::Vec3b>(xy_rect);
                    obj_cloud_ptr->push_back(pcl::PointXYZHSV(point_tf.x, point_tf.y, point_tf.z, ((float)hsv[0])*2, ((float)hsv[1])/((float)255.0), ((float)hsv[2])/((float)255.0)));
                    // obj_cloud_ptr->push_back(pcl::PointXYZRGB(point_tf.x, point_tf.y, point_tf.z, bgr[2], bgr[1], bgr[0]));
                    RCLCPP_DEBUG(this->get_logger(), "SELECTED HSV POINT PROJECTED ONTO IMAGE: (%lf, %lf) -> (%d, %d, %d)", xy_rect.x, xy_rect.y, hsv[0], hsv[1], hsv[2]);
                    // RCLCPP_DEBUG(this->get_logger(), "SELECTED RGB POINT PROJECTED ONTO IMAGE: (%lf, %lf) -> (%d, %d, %d)", xy_rect.x, xy_rect.y, bgr[2], bgr[1], bgr[0]);
                }
            }
        }
        pcl::toROSMsg(*obj_cloud_ptr, labeled_pcl.cloud);
        labeled_pcl.cloud.header.stamp = in_cloud_msg->header.stamp;
        object_pcls.objects.push_back(labeled_pcl);
        obj_cloud_vec.push_back(*obj_cloud_ptr);
        bbox_pcloud_objects.push_back(std::make_pair(bbox, obj_cloud_ptr));
        obj++;
        RCLCPP_DEBUG(this->get_logger(), "%d POINTS IN OBJECT %d", obj_cloud_ptr->size(), obj);
        // max_len = std::max(max_len, (int)obj_cloud_ptr->size());
    }
    RCLCPP_DEBUG(this->get_logger(), "WILL NOW SEND OBJECT POINT CLOUDS");
    m_object_pcl_pub->publish(object_pcls);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_obj_pcls_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    all_obj_pcls_ptr->header = in_cloud_tf_ptr->header;
    //convert vector of PointCloud to a single PointCloud with channels
    // all_obj_pcls_ptr->resize((pcl::uindex_t)max_len, (pcl::uindex_t)in_bbox_msg->boxes.size());
    // RCLCPP_DEBUG(this->get_logger(), "PUBLISHED PCLOUD DIMENSIONS: height: %d, width: %d", (int)all_obj_pcls_ptr->height, (int)all_obj_pcls_ptr->width);
    // RCLCPP_DEBUG(this->get_logger(), "STORED PCLOUD DIMENSIONS: objects: %d, max_length: %d", (int)obj_cloud_vec.size(), max_len);
    try{
        for(int i = 0; i<obj_cloud_vec.size(); i++){
            for(int j = 0; j<obj_cloud_vec[i].size(); j++){
                // all_obj_pcls_ptr->at(j,i) = obj_cloud_vec[i][j];
                all_obj_pcls_ptr->push_back(obj_cloud_vec[i][j]);
            }
        }
    }catch(std::exception &ex){
        RCLCPP_ERROR(this->get_logger(), "PROJECT BBOX PCLOUD PUBLISHING ERROR: %s", ex.what());
    }
    auto obj_pcls_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*all_obj_pcls_ptr, obj_pcls_msg);
    obj_pcls_msg.header.stamp = in_cloud_msg->header.stamp;
    m_object_pcl_viz_pub->publish(obj_pcls_msg);

    // REFINE OBJECT POINT CLOUDS
    auto refined_objects_pub = all_seaing_interfaces::msg::LabeledObjectPointCloudArray();
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZHSV>, std::vector<cv::Point>>> refined_cloud_contour_vec;
    // int max_refined_len = 0;

    RCLCPP_DEBUG(this->get_logger(), "# OF FOUND BBOXES: %d", bbox_pcloud_objects.size());
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

        //image & bbox relation & adjustment (make sure in-bounds)
        RCLCPP_DEBUG(this->get_logger(), "BBOX COORDS: (%d, %d), (%d, %d) in img of sz %d x %d", bbox.min_x, bbox.min_y, bbox.max_x+1, bbox.max_y+1, cv_hsv.cols, cv_hsv.rows);
        if(bbox.min_x > bbox.max_x || bbox.min_y > bbox.max_y) continue;
        //make sure bbox coords are inside image
        int adj_min_x = std::max((int)bbox.min_x,0);
        int adj_max_x = std::min((int)bbox.max_x+1, cv_hsv.cols);
        int adj_min_y = std::max((int)bbox.min_y,0);
        int adj_max_y = std::min((int)bbox.max_y+1, cv_hsv.rows);
        RCLCPP_DEBUG(this->get_logger(), "ADJUSTED BBOX COORDS: (%d, %d), (%d, %d) in img of sz %d x %d", adj_min_x, adj_min_y, adj_max_x, adj_max_y, cv_hsv.cols, cv_hsv.rows);
        cv::Mat cv_hsv_copy = cv_hsv.clone();
        cv::Mat hsv_img(cv_hsv, cv::Range(adj_min_y, adj_max_y), cv::Range(adj_min_x, adj_max_x));
        cv::Size img_sz;
        cv::Point bbox_offset;
        hsv_img.locateROI(img_sz, bbox_offset);
        RCLCPP_DEBUG(this->get_logger(), "IMAGE SIZE: (%d, %d)", img_sz.height, img_sz.width);
        RCLCPP_DEBUG(this->get_logger(), "BBOX OFFSET: (%d, %d)", bbox_offset.x, bbox_offset.y);

        //to display points inside contours, and the clusters projected (debugging)
        cv::Mat cv_img_copy = cv_ptr->image.clone();
        cv::Mat mat_clusters(cv_img_copy, cv::Range(adj_min_y, adj_max_y), cv::Range(adj_min_x, adj_max_x));
        cv::Mat mat_contours = mat_clusters.clone();
        cv::Mat mat_opt_cluster = mat_clusters.clone();
        cv::Mat mat_opt_contour = mat_contours.clone();

        //extract clusters
        pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>);
        // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
        std::vector<pcl::PointIndices> clusters_indices;

        // CONDITIONAL (WITH HSV-BASED CONDITION) EUCLIDEAN CLUSTERING
        pcl::ConditionalEuclideanClustering<pcl::PointXYZHSV> cec;
        cec.setClusterTolerance(m_clustering_distance);
        cec.setMinClusterSize(m_obstacle_size_min);
        cec.setMaxClusterSize(m_obstacle_size_max);
        cec.setSearchMethod(tree);
        cec.setInputCloud(pcloud_ptr);
        std::function<bool(const pcl::PointXYZHSV&, const pcl::PointXYZHSV&, float)> cond_func = std::bind(&hsv_diff_condition, m_clustering_color_weights, m_clustering_color_thres, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        cec.setConditionFunction(cond_func);
        cec.segment(clusters_indices);
        RCLCPP_DEBUG(this->get_logger(), "# OF EXTRACTED CLUSTERS: %d", clusters_indices.size());
        int clust_id = 0;
        for(auto ind_set : clusters_indices){
            RCLCPP_DEBUG(this->get_logger(), "SIZE OF CLUSTER %d: %d", clust_id, ind_set.indices.size());
            for(pcl::index_t ind : ind_set.indices){
                cv::Point2d cloud_pt_xy = m_is_sim ? m_cam_model.project3dToPixel(cv::Point3d(pcloud_ptr->points[ind].y, pcloud_ptr->points[ind].z, -pcloud_ptr->points[ind].x)) : m_cam_model.project3dToPixel(cv::Point3d(pcloud_ptr->points[ind].x, pcloud_ptr->points[ind].y, pcloud_ptr->points[ind].z));
                mat_clusters.at<cv::Vec3b>((cv::Point)cloud_pt_xy-bbox_offset) = int_to_bgr(clust_id, clusters_indices.size());
            }
            clust_id++;
        }
        //show received bbox
        // cv::Mat bgr_bbox;
        // cv::cvtColor(hsv_img, bgr_bbox, cv::COLOR_HSV2BGR);
        // cv::imshow("BBox", bgr_bbox);
        // cv::waitKey();

        // color segmentation using the color label
        std::vector<int> lims = label_color_map[bbox.label]=="red"?contour_matching_color_range_map["red2"]:contour_matching_color_range_map[label_color_map[bbox.label]];
        int h_min=lims[0], h_max=lims[1], s_min=lims[2], s_max=lims[3], v_min=lims[4], v_max=lims[5];
        RCLCPP_DEBUG(this->get_logger(), "COLOR: %s, & SEGMENTATION COLOR LIMITS-> H: %d - %d, S: %d - %d, V: %d - %d", label_color_map[bbox.label].c_str(), h_min, h_max, s_min, s_max, v_min, v_max);
        cv::Scalar bbox_mean_hsv = cv::mean(hsv_img);
        RCLCPP_DEBUG(this->get_logger(), "MEAN BBOX IMAGE COLOR: (%lf, %lf, %lf)", bbox_mean_hsv[0], bbox_mean_hsv[1], bbox_mean_hsv[2]);
        cv::Mat mask;
        cv::inRange(hsv_img, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);
        //show mask
        // cv::imshow("Mask", mask);
        // cv::waitKey();
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)));
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,7)));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> in_contours;
        //Get image points inside contour, put them into a vector, will then process those (not the contours themselves, they are just a boundary)
        for(std::vector<cv::Point> contour : contours){
            //TAKE THE BOUNDING BOX (BOUNDINGRECT), TAKE ALL THE PIXELS IN IT (NOT NOT CHECK EVERY PIXEL IN THE IMAGE) AND CHECK IF IT'S INSIDE THE CONTOUR USING POINTPOLYGONTEST
            cv::Rect cont_bbox = cv::boundingRect(contour);
            std::vector<cv::Point> in_contour;
            for(int x = cont_bbox.x; x < cont_bbox.x + cont_bbox.width; x++){
                for(int y = cont_bbox.y; y < cont_bbox.y + cont_bbox.height; y++){
                    cv::Point pt = cv::Point(x,y);
                    if(cv::pointPolygonTest(contour, pt, false)>=0){
                        in_contour.push_back(pt);
                    }
                }
            }
            in_contours.push_back(in_contour);
        }
        RCLCPP_DEBUG(this->get_logger(), "# OF FOUND CONTOURS: %d", in_contours.size());
        // Convert the contour points to fit the original image (using image_sz and bbox_offset -> just bbox_offset+pt_coords) to be able to use it with the LiDAR (projected) points
        for(int ctr = 0; ctr < in_contours.size(); ctr++){
            RCLCPP_DEBUG(this->get_logger(), "SIZE OF CONTOUR %d: %d", ctr, in_contours[ctr].size());
            for(int pt=0; pt < in_contours[ctr].size(); pt++){
                in_contours[ctr][pt]+=bbox_offset;
                mat_contours.at<cv::Vec3b>(in_contours[ctr][pt]-bbox_offset) = int_to_bgr(ctr, in_contours.size());
            }
        }
        if(in_contours.empty() || clusters_indices.empty()) continue;
        // Convert both the pcloud and the contours to a vector of pair<Point2d, Vec3b> and compute sum and sum of squares (can be used to compute all the needed metrics)
        std::vector<std::vector<std::pair<cv::Point2d, std::vector<long long>>>> contours_pts;
        std::vector<std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>>> contours_qts;
        for (std::vector<cv::Point> in_contour : in_contours){// point.x, point.y the coords
            std::vector<std::pair<cv::Point2d, std::vector<long long>>> contour_pts;
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> contour_qts;
            contour_qts.first.second = contour_qts.second.second = {0,0,0};
            for(cv::Point image_pt : in_contour){
                cv::Vec3b image_pt_hsv_vec3b = cv_hsv.at<cv::Vec3b>(image_pt);
                std::vector<long long> image_pt_hsv = {(long long)image_pt_hsv_vec3b[0], (long long)image_pt_hsv_vec3b[1], (long long)image_pt_hsv_vec3b[2]};
                RCLCPP_DEBUG(this->get_logger(), "IMAGE POINT COLOR AT POINT (%d, %d) (size: %d, %d): (%d, %d, %d)", image_pt.x, image_pt.y, cv_hsv.cols, cv_hsv.rows, image_pt_hsv[0], image_pt_hsv[1], image_pt_hsv[2]);
                cv::Point2d image_pt_xy = cv::Point2d(image_pt.x, image_pt.y);
                contour_pts.push_back(std::make_pair(image_pt_xy, image_pt_hsv));
                RCLCPP_DEBUG(this->get_logger(), "IMAGE H SUM/SQUARED BEFORE POINT UPDATE: %lld/%lld", contour_qts.first.second[0], contour_qts.second.second[0]);
                //store sums
                contour_qts.first.first.x+=image_pt_xy.x;
                contour_qts.first.first.y+=image_pt_xy.y;
                contour_qts.first.second[0]+=image_pt_hsv[0];
                contour_qts.first.second[1]+=image_pt_hsv[1];
                contour_qts.first.second[2]+=image_pt_hsv[2];
                //store sum of squares
                contour_qts.second.first.x+=image_pt_xy.x*image_pt_xy.x;
                contour_qts.second.first.y+=image_pt_xy.y*image_pt_xy.y;
                contour_qts.second.second[0]+=image_pt_hsv[0]*image_pt_hsv[0];
                contour_qts.second.second[1]+=image_pt_hsv[1]*image_pt_hsv[1];
                contour_qts.second.second[2]+=image_pt_hsv[2]*image_pt_hsv[2];
                RCLCPP_DEBUG(this->get_logger(), "IMAGE H SUM/SQUARED AFTER POINT UPDATE: %lld/%lld", contour_qts.first.second[0], contour_qts.second.second[0]);
            }
            contours_pts.push_back(contour_pts);
            contours_qts.push_back(contour_qts);
        }
        std::vector<std::vector<std::pair<cv::Point2d, std::vector<long long>>>> clusters_pts;
        std::vector<std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>>> clusters_qts;
        for (pcl::PointIndices cluster : clusters_indices){// vector<index_t> is cluster.indices
            std::vector<std::pair<cv::Point2d, std::vector<long long>>> cluster_pts;
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> cluster_qts;
            cluster_qts.first.second = cluster_qts.second.second = {0,0,0};
            for(pcl::index_t ind : cluster.indices){
                RCLCPP_DEBUG(this->get_logger(), "INITIAL CLUSTER POINT COLOR: (%lf, %lf, %lf)", pcloud_ptr->points[ind].h, pcloud_ptr->points[ind].s, pcloud_ptr->points[ind].v);
                //transform to OpenCV HSV
                std::vector<long long> cloud_pt_hsv = {pcloud_ptr->points[ind].h/2, pcloud_ptr->points[ind].s*((float)255.0), pcloud_ptr->points[ind].v*((float)255.0)};
                RCLCPP_DEBUG(this->get_logger(), "CONVERTED CLUSTER POINT COLOR: (%d, %d, %d)", cloud_pt_hsv[0], cloud_pt_hsv[1], cloud_pt_hsv[2]);
                cv::Point2d cloud_pt_xy = m_is_sim ? m_cam_model.project3dToPixel(cv::Point3d(pcloud_ptr->points[ind].y, pcloud_ptr->points[ind].z, -pcloud_ptr->points[ind].x)) : m_cam_model.project3dToPixel(cv::Point3d(pcloud_ptr->points[ind].x, pcloud_ptr->points[ind].y, pcloud_ptr->points[ind].z));
                cluster_pts.push_back(std::make_pair(cloud_pt_xy, cloud_pt_hsv));
                //store sums
                cluster_qts.first.first.x+=cloud_pt_xy.x;
                cluster_qts.first.first.y+=cloud_pt_xy.y;
                cluster_qts.first.second[0]+=cloud_pt_hsv[0];
                cluster_qts.first.second[1]+=cloud_pt_hsv[1];
                cluster_qts.first.second[2]+=cloud_pt_hsv[2];
                //store sum of squares
                cluster_qts.second.first.x+=cloud_pt_xy.x*cloud_pt_xy.x;
                cluster_qts.second.first.y+=cloud_pt_xy.y*cloud_pt_xy.y;
                cluster_qts.second.second[0]+=cloud_pt_hsv[0]*cloud_pt_hsv[0];
                cluster_qts.second.second[1]+=cloud_pt_hsv[1]*cloud_pt_hsv[1];
                cluster_qts.second.second[2]+=cloud_pt_hsv[2]*cloud_pt_hsv[2];
            }
            clusters_pts.push_back(cluster_pts);
            clusters_qts.push_back(cluster_qts);
        }

        // GO THROUGH ALL THE CLUSTER/CONTOUR PAIRS AND FIND THE BEST ONE BASED ON THE OPTIMALITY METRIC WITH THE WEIGHTS
        long long min_pair_cost = -1;
        int opt_contour_id = -1, opt_cluster_id = -1;
        for (int contour = 0; contour < contours_pts.size(); contour++){
            long long contour_size = contours_pts[contour].size();
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> contour_qts = contours_qts[contour];
            
            long long contour_cost = 0;
            for (std::pair<cv::Point2d, std::vector<long long>> contour_pt : contours_pts[contour]){
                contour_cost += (long long)color_range_penalty(m_contour_detection_color_weights, label_color_map[bbox.label]=="red"?contour_matching_color_range_map["red2"]:contour_matching_color_range_map[label_color_map[bbox.label]], contour_pt.second)/contour_size;
            }
            for (int cluster = 0; cluster < clusters_pts.size(); cluster++){
                long long cluster_size = clusters_pts[cluster].size();
                std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> cluster_qts = clusters_qts[cluster];
                
                //sum(cluster_size*x_i^2+contour_size*x_j^2) -2*sum(x_i*x_j) +sum(cluster_size*y_i^2+contour_size*y_j^2) -2*sum(y_i*y_j))
                long long fast_contour_cluster_sq_dist_sum = cluster_size*contour_qts.second.first.x+contour_size*cluster_qts.second.first.x\
                                        - 2*contour_qts.first.first.x*cluster_qts.first.first.x\
                                        + cluster_size*contour_qts.second.first.y+contour_size*cluster_qts.second.first.y\
                                        - 2*contour_qts.first.first.y*cluster_qts.first.first.y;
                long long contour_cluster_dist_ms = fast_contour_cluster_sq_dist_sum/(cluster_size*contour_size);

                std::vector<long long> avg_contour_col = {contour_qts.first.second[0]/contour_size, contour_qts.first.second[1]/contour_size, contour_qts.first.second[2]/contour_size};
  
                long long fast_cluster_sq_dh_sum = cluster_qts.second.second[0]-2*cluster_qts.first.second[0]*avg_contour_col[0]+cluster_size*avg_contour_col[0]*avg_contour_col[0];//sum((h_i-m_h)^2=h_i^2-2*h_i*m_h+cluster_size*m_h^2)
                long long fast_cluster_sq_ds_sum = cluster_qts.second.second[1]-2*cluster_qts.first.second[1]*avg_contour_col[1]+cluster_size*avg_contour_col[1]*avg_contour_col[1];//same for s
                long long fast_cluster_sq_dv_sum = cluster_qts.second.second[2]-2*cluster_qts.first.second[2]*avg_contour_col[2]+cluster_size*avg_contour_col[2]*avg_contour_col[2];//same for v

                long long cluster_dh_ms = fast_cluster_sq_dh_sum/cluster_size;
                long long cluster_ds_ms = fast_cluster_sq_ds_sum/cluster_size;
                long long cluster_dv_ms = fast_cluster_sq_dv_sum/cluster_size;
                
                long long pair_cost = m_cluster_contour_distance_weight*contour_cluster_dist_ms\
                                    + m_cluster_contour_color_weights[0]*cluster_dh_ms\
                                    + m_cluster_contour_color_weights[1]*cluster_ds_ms\
                                    + m_cluster_contour_color_weights[2]*cluster_dv_ms\
                                    - m_cluster_contour_size_weight*cluster_size*contour_size\
                                    + contour_cost;
                if (min_pair_cost == -1 || pair_cost < min_pair_cost){
                    min_pair_cost = pair_cost;
                    opt_contour_id = contour;
                    opt_cluster_id = cluster;
                }
            }
        }

        auto refined_pcl_segments = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        refined_pcl_segments.time = in_cloud_msg->header.stamp;
        refined_pcl_segments.label = bbox.label;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr refined_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        refined_cloud_ptr->header = pcloud_ptr->header;
        cv::Mat refined_obj_contour_mat = cv::Mat::zeros(img_sz, CV_8UC3);
        for (cv::Point image_pt : in_contours[opt_contour_id]){
            refined_obj_contour_mat.at<cv::Vec3b>(image_pt) = cv::Vec3b(255,255,255);
            mat_opt_contour.at<cv::Vec3b>(image_pt-bbox_offset) = int_to_bgr(opt_contour_id, in_contours.size());
        }
        for (pcl::index_t ind : clusters_indices[opt_cluster_id].indices){
            pcl::PointXYZHSV pt = pcloud_ptr->points[ind];
            refined_cloud_ptr->push_back(pt);
            cv::Point2d cloud_pt_xy = m_is_sim ? m_cam_model.project3dToPixel(cv::Point3d(pt.y, pt.z, -pt.x)) : m_cam_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));
            mat_opt_cluster.at<cv::Vec3b>((cv::Point)cloud_pt_xy-bbox_offset) = int_to_bgr(opt_cluster_id, clusters_indices.size());
        }
        pcl::toROSMsg(*refined_cloud_ptr, refined_pcl_segments.cloud);
        refined_pcl_segments.cloud.header.stamp = in_cloud_msg->header.stamp;
        cv_bridge::CvImagePtr refined_obj_contour_ptr(new cv_bridge::CvImage(in_img_msg->header, sensor_msgs::image_encodings::TYPE_8UC3, refined_obj_contour_mat));
        // cv::imshow("Object contour image to be published:", refined_obj_contour_mat);
        // cv::waitKey();
        refined_pcl_segments.segment = *refined_obj_contour_ptr->toImageMsg();
        refined_objects_pub.objects.push_back(refined_pcl_segments);
        refined_cloud_contour_vec.push_back(std::make_pair(*refined_cloud_ptr,in_contours[opt_contour_id]));
        // max_refined_len = std::max(max_refined_len, (int)refined_cloud_ptr->size());

        //show clusters & contours & respective matching (split image)
        // cv::Mat clust_cont_arr[] = {mat_clusters, mat_contours};
        // cv::Mat opt_clust_cont_arr[] = {mat_opt_cluster, mat_opt_contour};
        // cv::Mat clust_cont;
        // cv::Mat opt_clust_cont;
        // cv::hconcat(clust_cont_arr, 2, clust_cont);
        // cv::hconcat(opt_clust_cont_arr, 2, opt_clust_cont);
        // cv::Mat clust_cont_all_arr[] = {clust_cont, opt_clust_cont};
        // cv::Mat clust_cont_all;
        // cv::vconcat(clust_cont_all_arr, 2, clust_cont_all);
        // cv::Mat upscaled;
        // cv::resize(clust_cont_all, upscaled, cv::Size(), 5, 5, cv::INTER_CUBIC);
        // cv::imshow("Clusters & contours & matching", upscaled);
        // cv::waitKey();
    }
    // RCLCPP_DEBUG(this->get_logger(), "WILL NOW SEND REFINED OBJECT POINT CLOUDS & CONTOURS");
    m_refined_object_pcl_segment_pub->publish(refined_objects_pub);
    // RCLCPP_DEBUG(this->get_logger(), "PUBLISHED REFINED OBJECT POINT CLOUDS & CONTOURS");
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_refined_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    all_obj_refined_pcls_ptr->header = in_cloud_tf_ptr->header;
    //convert vector of PointCloud to a single PointCloud with channels
    // all_obj_refined_pcls_ptr->resize((pcl::uindex_t)max_refined_len, (pcl::uindex_t)refined_cloud_contour_vec.size());
    cv::Mat all_obj_refined_contours = cv::Mat::zeros(cv_ptr->image.size(), CV_8UC3);
    try{
        for(int i = 0; i<refined_cloud_contour_vec.size(); i++){
            RCLCPP_DEBUG(this->get_logger(), "BBOX %d/%d", i, refined_cloud_contour_vec.size());
            for(int j = 0; j<refined_cloud_contour_vec[i].first.size(); j++){
                // all_obj_refined_pcls_ptr->at(j,i) = refined_cloud_contour_vec[i].first[j];
                all_obj_refined_pcls_ptr->push_back(refined_cloud_contour_vec[i].first[j]);
            }
            for(cv::Point pt : refined_cloud_contour_vec[i].second){
                all_obj_refined_contours.at<cv::Vec3b>(pt)=cv::Vec3b(255, 255, 255);
            }
        }
    }catch(std::exception &ex){
        RCLCPP_ERROR(this->get_logger(), "REFINED POINT CLOUD PUBLISHING ERROR: %s", ex.what());
    }
    RCLCPP_DEBUG(this->get_logger(), "STORED CONTOURS TO BE PUBLISHED");
    auto obj_refined_pcls_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*all_obj_refined_pcls_ptr, obj_refined_pcls_msg);
    obj_refined_pcls_msg.header.stamp = in_cloud_msg->header.stamp;
    m_refined_object_pcl_viz_pub->publish(obj_refined_pcls_msg);
    cv_bridge::CvImagePtr all_obj_refined_contour_ptr(new cv_bridge::CvImage(in_img_msg->header, sensor_msgs::image_encodings::TYPE_8UC3, all_obj_refined_contours));
    // cv::imshow("Object contour image to be published:", all_obj_refined_contours);
    // cv::waitKey();
    m_refined_object_segment_viz_pub->publish(*all_obj_refined_contour_ptr->toImageMsg());
    // RCLCPP_DEBUG(this->get_logger(), "SENT OBJECT POINT CLOUDS FOR VISUALIZATION");
}

geometry_msgs::msg::TransformStamped BBoxProjectPCloud::get_tf(const std::string &in_target_frame,
                                                             const std::string &in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    m_pc_cam_tf_ok = false;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        m_pc_cam_tf_ok = true;
        RCLCPP_DEBUG(this->get_logger(), "LiDAR to Camera Transform good");
        RCLCPP_DEBUG(this->get_logger(), "in_target_frame: %s, in_src_frame: %s",
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