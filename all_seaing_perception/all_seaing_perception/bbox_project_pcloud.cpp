#include "all_seaing_perception/bbox_project_pcloud.hpp"

BBoxProjectPCloud::BBoxProjectPCloud() : Node("bbox_project_pcloud"){
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    //essential ones
    this->declare_parameter<double>("bbox_object_margin", 0.0);
    m_bbox_margin = this->get_parameter("bbox_object_margin").as_double();

    this->declare_parameter<double>("contour_bbox_area_thres", 0.5);
    m_contour_bbox_area_thres = this->get_parameter("contour_bbox_area_thres").as_double();

    this->declare_parameter<double>("cluster_bbox_area_thres", 0.35);
    m_cluster_bbox_area_thres = this->get_parameter("cluster_bbox_area_thres").as_double();

    this->declare_parameter<double>("overlap_filter_thres", 0.3);
    m_overlap_filter_thres = this->get_parameter("overlap_filter_thres").as_double();

    // for cluster extraction
    this->declare_parameter<int>("obstacle_size_min", 20);
    this->declare_parameter<int>("obstacle_size_max", 100000);
    this->declare_parameter<double>("clustering_distance", 0.75);
    
    m_obstacle_size_min = this->get_parameter("obstacle_size_min").as_int();
    m_obstacle_size_max = this->get_parameter("obstacle_size_max").as_int();
    m_clustering_distance = this->get_parameter("clustering_distance").as_double();

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    RCLCPP_INFO(this->get_logger(), m_is_sim?"IS SIM: TRUE":"IS SIM: FALSE");

    this->declare_parameter<bool>("label_list", true);
    m_label_list = this->get_parameter("label_list").as_bool();

    // for color segmentation
    this->declare_parameter("color_label_mappings_file", "");

    color_label_mappings_file = this->get_parameter("color_label_mappings_file").as_string();

    this->declare_parameter("base_link_frame", "base_link");
    m_base_link_frame = this->get_parameter("base_link_frame").as_string();

    // for cluster-contour matching & selection
    this->declare_parameter("matching_weights_file", "");
    this->declare_parameter("contour_matching_color_ranges_file", "");

    matching_weights_file = this->get_parameter("matching_weights_file").as_string();
    contour_matching_color_ranges_file = this->get_parameter("contour_matching_color_ranges_file").as_string();

    this->declare_parameter<std::string>("camera_name", "");
    m_camera_name = this->get_parameter("camera_name").as_string();

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
    m_object_pcl_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>(m_camera_name!=""?std::string("labeled_object_point_clouds/")+m_camera_name:"labeled_object_point_clouds", 5);
    m_object_pcl_viz_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_camera_name!=""?std::string("object_point_clouds_viz/")+m_camera_name:"object_point_clouds_viz", 5);
    m_detection_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(m_camera_name!=""?std::string("detections/")+m_camera_name:"detections", 5);
    m_refined_object_pcl_viz_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_camera_name!=""?std::string("refined_object_point_clouds_viz/")+m_camera_name:"refined_object_point_clouds_viz", 5);

    // get color label mappings from yaml
    std::ifstream label_yaml(color_label_mappings_file);
    if (label_yaml.is_open()) {
        label_config_yaml = YAML::Load(label_yaml);  
        for (YAML::const_iterator it = label_config_yaml.begin(); it != label_config_yaml.end(); ++it) {
            if(m_label_list){
                for(int label : it->second.as<std::vector<int>>()){
                    label_color_map[label] = it->first.as<std::string>();
                    RCLCPP_INFO(this->get_logger(), "%d -> %s", label, it->first.as<std::string>().c_str());
                }
            }else{
                label_color_map[it->second.as<int>()] = it->first.as<std::string>();
                RCLCPP_INFO(this->get_logger(), "%d -> %s", it->second.as<int>(), it->first.as<std::string>().c_str());
            }
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", color_label_mappings_file.c_str());
    }
    label_yaml.close();

    // get cluster-contour matching & selection parameters from yaml
    std::ifstream matching_yaml(matching_weights_file);
    if (matching_yaml.is_open()) {
        matching_weights_config_yaml = YAML::Load(matching_yaml);

        m_clustering_color_weights = matching_weights_config_yaml["clustering_color_weights"].as<std::vector<double>>();
        m_clustering_color_thres = matching_weights_config_yaml["clustering_color_thres"].as<double>();
        m_cluster_contour_distance_weight = matching_weights_config_yaml["cluster_contour_distance_weight"].as<double>();
        m_cluster_contour_color_weights = matching_weights_config_yaml["cluster_contour_color_weights"].as<std::vector<double>>();
        m_contour_detection_color_weights = matching_weights_config_yaml["contour_detection_color_weights"].as<std::vector<double>>();
        m_cluster_contour_size_weight = matching_weights_config_yaml["cluster_contour_size_weight"].as<double>();
        m_cluster_distance_weight = matching_weights_config_yaml["cluster_distance_weight"].as<double>();
        m_cluster_area_ratio_weight = matching_weights_config_yaml["cluster_area_ratio_weight"].as<double>();
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", matching_weights_file.c_str());
    }
    matching_yaml.close();

    // get contour matching color ranges from yaml
    std::ifstream contour_ranges_yaml(contour_matching_color_ranges_file);
    if (contour_ranges_yaml.is_open()) {
        contour_matching_ranges_config_yaml = YAML::Load(contour_ranges_yaml);  
        for (YAML::const_iterator it = contour_matching_ranges_config_yaml.begin(); it != contour_matching_ranges_config_yaml.end(); ++it) {
            contour_matching_color_range_map[it->first.as<std::string>()] = it->second.as<std::vector<int>>();
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", contour_matching_color_ranges_file.c_str());
    }
    contour_ranges_yaml.close();
}

void BBoxProjectPCloud::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    m_cam_model.fromCameraInfo(info_msg);
}

// The HSV-similarity condition for Conditional Euclidean Clustering
bool hsv_diff_condition(std::vector<double> weights, double thres, const pcl::PointXYZHSV& p1, const pcl::PointXYZHSV& p2, float sq_dist){
    return weights[0]*(p1.h-p2.h)*(p1.h-p2.h)+weights[1]*(p1.s-p2.s)*(p1.s-p2.s)+weights[2]*(p1.v-p2.v)*(p1.v-p2.v) < thres*thres;
}

double line_range_penalty(int a, int b, int x){
    // Average of squared distances from each point in the range
    return (a<=x && x<=b)?0:(pow(b-x,3)-pow(a-x,3))/(b-a);
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

double overlap(all_seaing_interfaces::msg::LabeledBoundingBox2D bbox1,all_seaing_interfaces::msg::LabeledBoundingBox2D bbox2){
    double min_x = std::max(bbox1.min_x, bbox2.min_x);
    double min_y = std::max(bbox1.min_y, bbox2.min_y);
    double max_x = std::min(bbox1.max_x, bbox2.max_x);
    double max_y = std::min(bbox1.max_y, bbox2.max_y);
    if (min_x >= max_x || min_y >= max_y){
        return 0;
    }else{
        return (max_x-min_x)*(max_y-min_y);
    }
}

double area(all_seaing_interfaces::msg::LabeledBoundingBox2D bbox){
    return (bbox.max_x-bbox.min_x)*(bbox.max_y-bbox.min_y);
}

void BBoxProjectPCloud::bb_pcl_project(
    const sensor_msgs::msg::Image::ConstSharedPtr &in_img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &in_cloud_msg,
    const all_seaing_interfaces::msg::LabeledBoundingBox2DArray::ConstSharedPtr &in_bbox_msg) {

    // LIDAR -> Camera transform (useful for projecting the camera bboxes onto the point cloud, have the origin on the camera frame)
    if (!m_pc_cam_tf_ok){
        m_pc_cam_tf = get_tf(in_img_msg->header.frame_id, in_cloud_msg->header.frame_id, "pc_cam");
        if (!m_pc_cam_tf_ok)
            return;
    }
    // Camera -> base_link
    if (!m_cam_base_link_tf_ok){
        m_cam_base_link_tf = get_tf(m_base_link_frame, in_img_msg->header.frame_id, "cam_base_link");
        if (!m_cam_base_link_tf_ok)
            return;
    }

    // // measuring time
    // using std::chrono::high_resolution_clock;
    // using std::chrono::duration_cast;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

    // auto t1 = high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>), in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud_msg, *in_cloud_ptr);
    
    all_seaing_perception::transformPCLCloud(*in_cloud_ptr, *in_cloud_tf_ptr, m_pc_cam_tf);
    
    // auto t2_tf = high_resolution_clock::now();

    // duration<double, std::milli> ms_double_tf = t2_tf - t1_tf;
    
    // RCLCPP_INFO(this->get_logger(), "TRANSFORM TIME: %lfms", ms_double_tf.count());

    auto object_pcls = all_seaing_interfaces::msg::LabeledObjectPointCloudArray();
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>> obj_cloud_vec;
    // Just use the same pcloud to image projection, but check if it's within some binding box and assign it to that detection
    int obj = 0;
    // Convert msg to CvImage to work with CV2. Copy img since we will be modifying.
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(in_img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat cv_hsv;
    
    cv::cvtColor(cv_ptr->image, cv_hsv, cv::COLOR_BGR2HSV);

    std::vector<std::pair<all_seaing_interfaces::msg::LabeledBoundingBox2D, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>> bbox_pcloud_objects;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    all_obj_pcls_ptr->header = in_cloud_tf_ptr->header;

    std::vector<all_seaing_interfaces::msg::LabeledBoundingBox2D> sorted_bboxes = in_bbox_msg->boxes;
    std::vector<all_seaing_interfaces::msg::LabeledBoundingBox2D> filtered_bboxes;

    if (!sorted_bboxes.empty()){
        // filter from the smallest area boxes to the largest ones, since when we reach a box it won't be able to remove any other boxes anyways
        // TODO: When checking overlaps, don't check overlap with e.g. shapes, they are usually on the background
        std::sort(sorted_bboxes.begin(), sorted_bboxes.end(), [](all_seaing_interfaces::msg::LabeledBoundingBox2D a, all_seaing_interfaces::msg::LabeledBoundingBox2D b){return area(a) < area(b);});

        for (int i = 0; i < sorted_bboxes.size()-1; i++){
            // RCLCPP_INFO(this->get_logger(), "BBOX %d AREA: %lf", i, area(sorted_bboxes[i]));
            bool should_filter = false;
            for (int j = i+1; j < sorted_bboxes.size(); j++){
                // RCLCPP_INFO(this->get_logger(), "BBOX %d,%d OVERLAP: %lf, RATIO: %lf", i, j, overlap(sorted_bboxes[i], sorted_bboxes[j]), overlap(sorted_bboxes[i], sorted_bboxes[j])/area(sorted_bboxes[i]));
                if (overlap(sorted_bboxes[i], sorted_bboxes[j])/area(sorted_bboxes[i]) > m_overlap_filter_thres){
                    // RCLCPP_INFO(this->get_logger(), "FILTERED");
                    should_filter = true;
                    break;
                }
            }
            if (should_filter) continue;
            filtered_bboxes.push_back(sorted_bboxes[i]);
        }

        filtered_bboxes.push_back(sorted_bboxes.back());
    }
    m_local_header.stamp = in_cloud_msg->header.stamp;
    m_local_header.frame_id = m_base_link_frame;
    object_pcls.header = m_local_header;
    for (all_seaing_interfaces::msg::LabeledBoundingBox2D bbox : filtered_bboxes){
        auto labeled_pcl = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        labeled_pcl.time = in_cloud_msg->header.stamp;
        labeled_pcl.label = bbox.label;
        labeled_pcl.camera_name = m_camera_name;
        obj_cloud_ptr->header = in_cloud_tf_ptr->header;
        // Add padding to bbox
        all_seaing_perception::addBBoxPadding(bbox, m_bbox_margin, cv_hsv.rows, cv_hsv.cols);
        all_seaing_perception::PCLInBBoxHSV(in_cloud_tf_ptr, obj_cloud_ptr, bbox, cv_hsv, m_cam_model, m_is_sim);
        if (obj_cloud_ptr->points.empty()) continue;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr_tf(new pcl::PointCloud<pcl::PointXYZHSV>);
        all_seaing_perception::transformPCLCloud(*obj_cloud_ptr, *obj_cloud_ptr_tf, m_cam_base_link_tf);
        pcl::toROSMsg(*obj_cloud_ptr_tf, labeled_pcl.cloud);
        labeled_pcl.cloud.header.stamp = in_cloud_msg->header.stamp;
        object_pcls.objects.push_back(labeled_pcl);
        obj_cloud_vec.push_back(*obj_cloud_ptr);
        bbox_pcloud_objects.push_back(std::make_pair(bbox, obj_cloud_ptr));
        *all_obj_pcls_ptr += *obj_cloud_ptr;
        obj++;
    }
    m_object_pcl_pub->publish(object_pcls);
    auto obj_pcls_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*all_obj_pcls_ptr, obj_pcls_msg);
    obj_pcls_msg.header.stamp = in_cloud_msg->header.stamp;
    m_object_pcl_viz_pub->publish(obj_pcls_msg);
    
    // REFINE OBJECT POINT CLOUDS
    auto refined_objects_msg = all_seaing_interfaces::msg::ObstacleMap();
    refined_objects_msg.local_header = m_local_header;
    refined_objects_msg.ns = "labeled";
    refined_objects_msg.is_labeled = true;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_refined_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    all_obj_refined_pcls_ptr->header = in_cloud_tf_ptr->header;

    int id = 0;
    for(auto bbox_pcloud_pair : bbox_pcloud_objects){
        all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = bbox_pcloud_pair.first;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr = bbox_pcloud_pair.second;
        if(!label_color_map.count(bbox.label)) continue; //ignore objects that are not registered buoy types

        // image & bbox relation & adjustment (make sure in-bounds)
        if(bbox.min_x > bbox.max_x || bbox.min_y > bbox.max_y) continue;

        cv::Mat hsv_img(cv_hsv, cv::Range(bbox.min_y, bbox.max_y), cv::Range(bbox.min_x, bbox.max_x));
        cv::Size img_sz;
        cv::Point bbox_offset;
        hsv_img.locateROI(img_sz, bbox_offset);

        // auto t1_cluster = high_resolution_clock::now();

        if (label_color_map[bbox.label]=="red"){
            // invert colors of cloud if the label is red, to have red (now cyan) points close to each other in HSV range
            // PCL color range: (360,1,1)
            for (pcl::PointXYZHSV& pt : pcloud_ptr->points){
                all_seaing_perception::invertHSVPCL(pt);
            }
        }

        // extract clusters
        pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>);
        if (!pcloud_ptr->points.empty())
            tree->setInputCloud(pcloud_ptr);
        std::vector<pcl::PointIndices> clusters_indices;

        // CONDITIONAL (WITH HSV-BASED CONDITION) EUCLIDEAN CLUSTERING
        all_seaing_perception::euclideanClustering(pcloud_ptr, clusters_indices, m_clustering_distance, m_obstacle_size_min, m_obstacle_size_max, true, std::bind(&hsv_diff_condition, m_clustering_color_weights, m_clustering_color_thres, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        if (label_color_map[bbox.label]=="red"){
            // invert colors back
            for (pcl::PointXYZHSV& pt : pcloud_ptr->points){
                all_seaing_perception::invertHSVPCL(pt);
            }
        }

        // color segmentation
        cv::Mat mask;
        std::vector<std::vector<cv::Point>> contours;
        std::tie(mask, contours) = all_seaing_perception::colorSegmentationHSV(hsv_img, contour_matching_color_range_map[label_color_map[bbox.label]], 5, 7, label_color_map[bbox.label]=="red", contour_matching_color_range_map.count("red2")?contour_matching_color_range_map["red2"]:std::vector<int>());

        for(int ctr = 0; ctr < contours.size(); ctr++){
            for(int pt=0; pt < contours[ctr].size(); pt++){
                contours[ctr][pt]+=bbox_offset;
            }
        }

        if(contours.empty() || clusters_indices.empty()) continue;

        // pre-filter clusters & segments
        std::vector<std::vector<cv::Point>> contours_filtered;
        for (std::vector<cv::Point> contour : contours){
            if (cv::contourArea(contour)/area(bbox) > m_contour_bbox_area_thres){
                contours_filtered.push_back(contour);
            }
        }
        
        std::vector<pcl::PointIndices> clusters_indices_filtered;
        for (pcl::PointIndices cluster : clusters_indices){
            double min_x = bbox.max_x, min_y = bbox.max_y, max_x = 0, max_y = 0;
            for(pcl::index_t ind : cluster.indices){
                cv::Point2d cloud_pt_xy = all_seaing_perception::projectPCLPtToPixel(m_cam_model, pcloud_ptr->points[ind], m_is_sim);
                min_x = std::min(min_x, cloud_pt_xy.x);
                max_x = std::max(max_x, cloud_pt_xy.x);
                min_y = std::min(min_y, cloud_pt_xy.y);
                max_y = std::max(max_y, cloud_pt_xy.y);
            }
            if (((max_x-min_x)*(max_y-min_y))/area(bbox) > m_cluster_bbox_area_thres){
                clusters_indices_filtered.push_back(cluster);
            }
        }

        if(contours_filtered.empty() || clusters_indices_filtered.empty()) continue;
        
        // Convert both the pcloud and the contours to a vector of pair<Point2d, Vec3b> and compute sum and sum of squares (can be used to compute all the needed metrics)
        // std::vector<std::vector<std::pair<cv::Point2d, std::vector<long long>>>> contours_pts;
        std::vector<std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>>> contours_qts;
        std::vector<int> contour_szs;
        for (std::vector<cv::Point> contour : contours_filtered){
            cv::Moments mu = cv::moments(contour);
            // RCLCPP_INFO(this->get_logger(), "COMPUTED MOMENTS: %lf, %lf, %lf, %lf, %lf", mu.m00, mu.m10, mu.m01, mu.m20, mu.m02);
            contour_szs.push_back(mu.m00);
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> contour_qts;
            contour_qts.first.second = contour_qts.second.second = {0,0,0};
            contour_qts.first.first.x = mu.m10;
            contour_qts.first.first.y = mu.m01;
            contour_qts.second.first.x = mu.m20;
            contour_qts.second.first.y = mu.m02;
            // find correct average hsv
            std::vector<int> adjusted_color_range;
            if (label_color_map[bbox.label]!="red"){
                adjusted_color_range = contour_matching_color_range_map[label_color_map[bbox.label]];
            }else{
                adjusted_color_range = contour_matching_color_range_map["red"]; //(Hmin,Hmax, ...)
                adjusted_color_range[1] += 90; // Hmax from red is low, add 90
                adjusted_color_range[1] = contour_matching_color_range_map["red2"][0]-90; // Hmin from red2 is high, add 90 and subtract 180 -> -90
            }
            //approximate to make things faster
            contour_qts.first.second[0] = mu.m00*(long long)(float(adjusted_color_range[0]+adjusted_color_range[1])/2);
            contour_qts.first.second[1] = mu.m00*(long long)(float(adjusted_color_range[2]+adjusted_color_range[3])/2);
            contour_qts.first.second[2] = mu.m00*(long long)(float(adjusted_color_range[4]+adjusted_color_range[5])/2);
            contour_qts.second.second[0] = mu.m00*(long long)(float(adjusted_color_range[0]+adjusted_color_range[1])/2)*(long long)(float(adjusted_color_range[0]+adjusted_color_range[1])/2);
            contour_qts.second.second[1] = mu.m00*(long long)(float(adjusted_color_range[2]+adjusted_color_range[3])/2)*(long long)(float(adjusted_color_range[2]+adjusted_color_range[3])/2);
            contour_qts.second.second[2] = mu.m00*(long long)(float(adjusted_color_range[4]+adjusted_color_range[5])/2)*(long long)(float(adjusted_color_range[4]+adjusted_color_range[5])/2);
            contours_qts.push_back(contour_qts);
        }
        std::vector<std::vector<std::pair<cv::Point2d, std::vector<long long>>>> clusters_pts;
        std::vector<std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>>> clusters_qts;
        std::vector<double> cluster_dists;
        std::vector<double> cluster_area_ratios;
        for (pcl::PointIndices cluster : clusters_indices_filtered){// vector<index_t> is cluster.indices
            std::vector<std::pair<cv::Point2d, std::vector<long long>>> cluster_pts;
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> cluster_qts;
            cluster_qts.first.second = cluster_qts.second.second = {0,0,0};
            double min_x = bbox.max_x, min_y = bbox.max_y, max_x = 0, max_y = 0;
            for(pcl::index_t ind : cluster.indices){
                // transform to OpenCV HSV (180, 255, 255)
                std::vector<long long> cloud_pt_hsv = all_seaing_perception::HSVPCLToOpenCV<long long>(pcloud_ptr->points[ind]);
                if (label_color_map[bbox.label]=="red"){
                    all_seaing_perception::invertHSVOpenCV(cloud_pt_hsv); // shift the scale so that red values are close to one another -> invert colors, red is cyan now
                }
                cv::Point2d cloud_pt_xy = all_seaing_perception::projectPCLPtToPixel(m_cam_model, pcloud_ptr->points[ind], m_is_sim);
                cluster_pts.push_back(std::make_pair(cloud_pt_xy, cloud_pt_hsv));
                min_x = std::min(min_x, cloud_pt_xy.x);
                max_x = std::max(max_x, cloud_pt_xy.x);
                min_y = std::min(min_y, cloud_pt_xy.y);
                max_y = std::max(max_y, cloud_pt_xy.y);
                // store sums
                cluster_qts.first.first.x+=cloud_pt_xy.x;
                cluster_qts.first.first.y+=cloud_pt_xy.y;
                cluster_qts.first.second[0]+=cloud_pt_hsv[0];
                cluster_qts.first.second[1]+=cloud_pt_hsv[1];
                cluster_qts.first.second[2]+=cloud_pt_hsv[2];
                // store sum of squares
                cluster_qts.second.first.x+=cloud_pt_xy.x*cloud_pt_xy.x;
                cluster_qts.second.first.y+=cloud_pt_xy.y*cloud_pt_xy.y;
                cluster_qts.second.second[0]+=cloud_pt_hsv[0]*cloud_pt_hsv[0];
                cluster_qts.second.second[1]+=cloud_pt_hsv[1]*cloud_pt_hsv[1];
                cluster_qts.second.second[2]+=cloud_pt_hsv[2]*cloud_pt_hsv[2];
            }
            clusters_pts.push_back(cluster_pts);
            clusters_qts.push_back(cluster_qts);
            // compute cluster centroid
            pcl::PointXYZHSV centr;
            pcl::computeCentroid(*pcloud_ptr, cluster.indices, centr);
            cluster_dists.push_back(centr.x*centr.x+centr.y*centr.y+centr.z*centr.z);
            // RCLCPP_INFO(this->get_logger(), "(%lf, %lf)->(%lf, %lf) from (%lf, %lf)->(%lf, %lf)", min_x, min_y, max_x, max_y, (double)bbox.min_x, (double)bbox.min_y, (double)bbox.max_x, (double)bbox.max_y);
            cluster_area_ratios.push_back(((max_x-min_x)*(max_y-min_y))/area(bbox));
        }

        // GO THROUGH ALL THE CLUSTER/CONTOUR PAIRS AND FIND THE BEST ONE BASED ON THE OPTIMALITY METRIC WITH THE WEIGHTS
        long long min_pair_cost = -1;
        int opt_contour_id = -1, opt_cluster_id = -1;
        for (int contour = 0; contour < contours_qts.size(); contour++){
            long long contour_size = contour_szs[contour];
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> contour_qts = contours_qts[contour];
            
            long long contour_cost = 0;
            
            // RCLCPP_INFO(this->get_logger(), "contour cost: %ld", contour_cost);
            for (int cluster = 0; cluster < clusters_pts.size(); cluster++){
                long long cluster_size = clusters_pts[cluster].size();
                std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> cluster_qts = clusters_qts[cluster];
                
                // TODO: Change this, it penalizes large clusters (think of it same as moment of inertia, it's RMS within cluster, which is bigger with size, ^2 + distance between cluster & segment ^2) -> replace with distance between midpoints?
                // sum(cluster_size*x_i^2+contour_size*x_j^2) -2*sum(x_i*x_j) +sum(cluster_size*y_i^2+contour_size*y_j^2) -2*sum(y_i*y_j))
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

                double cluster_dist = cluster_dists[cluster];
                double cluster_area_ratio = cluster_area_ratios[cluster];

                // RCLCPP_INFO(this->get_logger(), "sqdist: %ld, dhsv: %ld, %ld, %ld, cluster size: %d, contour size: %d, dist: %lf, cluster area ratio: %lf, weights: %ld, %ld, %ld, %ld, %ld, %ld, %ld", contour_cluster_dist_ms, cluster_dh_ms, cluster_ds_ms, cluster_dv_ms, cluster_size, contour_size, (double)cluster_dist, cluster_area_ratio, (long long)m_cluster_contour_distance_weight, (long long)m_cluster_contour_color_weights[0], (long long)m_cluster_contour_color_weights[1], (long long)m_cluster_contour_color_weights[2], (long long)m_cluster_contour_size_weight, (long long)m_cluster_distance_weight, (long long)m_cluster_area_ratio_weight);
                
                long long pair_cost = m_cluster_contour_distance_weight*contour_cluster_dist_ms\
                                    + m_cluster_contour_color_weights[0]*cluster_dh_ms\
                                    + m_cluster_contour_color_weights[1]*cluster_ds_ms\
                                    + m_cluster_contour_color_weights[2]*cluster_dv_ms\
                                    - m_cluster_contour_size_weight*cluster_size*contour_size\
                                    + m_cluster_distance_weight*cluster_dist\
                                    - m_cluster_area_ratio_weight*cluster_area_ratio\
                                    + contour_cost;

                // RCLCPP_INFO(this->get_logger(), "pair cost: %ld", pair_cost);
                if (min_pair_cost == -1 || pair_cost < min_pair_cost){
                    min_pair_cost = pair_cost;
                    // opt_contour_id = contour;
                    opt_cluster_id = cluster;
                }
            }
        }

        // RCLCPP_INFO(this->get_logger(), "min pair cost: %ld", min_pair_cost);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr refined_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        refined_cloud_ptr->header = pcloud_ptr->header;
        for (pcl::index_t ind : clusters_indices_filtered[opt_cluster_id].indices){
            pcl::PointXYZHSV pt = pcloud_ptr->points[ind];
            refined_cloud_ptr->push_back(pt);
        }
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr refined_cloud_base_link_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);

        all_seaing_perception::transformPCLCloud(*refined_cloud_ptr, *refined_cloud_base_link_ptr, m_cam_base_link_tf);
                
        all_seaing_perception::Obstacle<pcl::PointXYZHSV> obstacle(m_local_header, refined_cloud_base_link_ptr, id++, false);
        obstacle.local_to_global(m_local_header, geometry_msgs::msg::TransformStamped());
        all_seaing_interfaces::msg::Obstacle obstacle_msg;
        obstacle.to_ros_msg(obstacle_msg);
        obstacle_msg.label = bbox.label;

        refined_objects_msg.obstacles.push_back(obstacle_msg);
        
        *all_obj_refined_pcls_ptr += *refined_cloud_ptr;
    }

    m_detection_pub->publish(refined_objects_msg);
    
    auto obj_refined_pcls_msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*all_obj_refined_pcls_ptr, obj_refined_pcls_msg);
    obj_refined_pcls_msg.header.stamp = in_cloud_msg->header.stamp;
    m_refined_object_pcl_viz_pub->publish(obj_refined_pcls_msg);
}

geometry_msgs::msg::TransformStamped BBoxProjectPCloud::get_tf(const std::string &in_target_frame,
                                                             const std::string &in_src_frame, std::string info) {
    geometry_msgs::msg::TransformStamped tf;
    if (info == "pc_cam")
        m_pc_cam_tf_ok = false;
    else if (info == "cam_base_link")
        m_cam_base_link_tf_ok = false;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
        if (info == "pc_cam")
            m_pc_cam_tf_ok = true;
        else if (info == "cam_base_link")
            m_cam_base_link_tf_ok = true;
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