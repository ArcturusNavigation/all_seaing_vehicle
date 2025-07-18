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

    this->declare_parameter<bool>("label_list", true);
    m_label_list = this->get_parameter("label_list").as_bool();

    this->declare_parameter<bool>("only_project", false);
    m_only_project = this->get_parameter("only_project").as_bool();

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

    // Transform in_cloud_msg to the camera frame and convert PointCloud2 to PCL PointCloud
    sensor_msgs::msg::PointCloud2 in_cloud_tf;
    tf2::doTransform<sensor_msgs::msg::PointCloud2>(*in_cloud_msg, in_cloud_tf, m_pc_cam_tf);
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_tf_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_tf, *in_cloud_tf_ptr);

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
    for (all_seaing_interfaces::msg::LabeledBoundingBox2D bbox : in_bbox_msg->boxes){
        if(!label_color_map.count(bbox.label)) continue; //ignore objects that are not registered buoy types

        auto labeled_pcl = all_seaing_interfaces::msg::LabeledObjectPointCloud();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        labeled_pcl.time = in_cloud_msg->header.stamp;
        labeled_pcl.label = bbox.label;
        labeled_pcl.camera_name = m_camera_name;
        obj_cloud_ptr->header = in_cloud_tf_ptr->header;
        // Add padding to bbox
        all_seaing_perception::addBBoxPadding(bbox, m_bbox_margin, cv_hsv.rows, cv_hsv.cols);
        all_seaing_perception::PCLInBBoxHSV(in_cloud_tf_ptr, obj_cloud_ptr, bbox, cv_hsv, m_cam_model, m_is_sim);
        pcl::toROSMsg(*obj_cloud_ptr, labeled_pcl.cloud);
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

    if(m_only_project) return;

    // REFINE OBJECT POINT CLOUDS
    auto refined_objects_msg = all_seaing_interfaces::msg::ObstacleMap();
    m_local_header.stamp = in_cloud_msg->header.stamp;
    m_local_header.frame_id = m_base_link_frame;
    refined_objects_msg.local_header = m_local_header;
    refined_objects_msg.ns = "labeled";
    refined_objects_msg.is_labeled = true;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr all_obj_refined_pcls_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
    all_obj_refined_pcls_ptr->header = in_cloud_tf_ptr->header;

    int id = 0;
    for(auto bbox_pcloud_pair : bbox_pcloud_objects){
        all_seaing_interfaces::msg::LabeledBoundingBox2D bbox = bbox_pcloud_pair.first;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcloud_ptr = bbox_pcloud_pair.second;

        // image & bbox relation & adjustment (make sure in-bounds)
        if(bbox.min_x > bbox.max_x || bbox.min_y > bbox.max_y) continue;

        cv::Mat hsv_img(cv_hsv, cv::Range(bbox.min_y, bbox.max_y), cv::Range(bbox.min_x, bbox.max_x));
        cv::Size img_sz;
        cv::Point bbox_offset;
        hsv_img.locateROI(img_sz, bbox_offset);

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
        std::vector<std::vector<cv::Point>> in_contours;
        // Get image points inside contour, put them into a vector, will then process those (not the contours themselves, they are just a boundary)
        for(std::vector<cv::Point> contour : contours){
            in_contours.push_back(all_seaing_perception::inContour(contour));
        }
        // Convert the contour points to fit the original image (using image_sz and bbox_offset -> just bbox_offset+pt_coords) to be able to use it with the LiDAR (projected) points
        for(int ctr = 0; ctr < in_contours.size(); ctr++){
            for(int pt=0; pt < in_contours[ctr].size(); pt++){
                in_contours[ctr][pt]+=bbox_offset;
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
                if (label_color_map[bbox.label]=="red"){
                    all_seaing_perception::invertHSVOpenCV(image_pt_hsv); // shift the scale so that red values are close to one another -> invert colors, red is cyan now
                }
                cv::Point2d image_pt_xy = cv::Point2d(image_pt.x, image_pt.y);
                contour_pts.push_back(std::make_pair(image_pt_xy, image_pt_hsv));
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
                // transform to OpenCV HSV (180, 255, 255)
                std::vector<long long> cloud_pt_hsv = all_seaing_perception::HSVPCLToOpenCV<long long>(pcloud_ptr->points[ind]);
                if (label_color_map[bbox.label]=="red"){
                    all_seaing_perception::invertHSVOpenCV(cloud_pt_hsv); // shift the scale so that red values are close to one another -> invert colors, red is cyan now
                }
                cv::Point2d cloud_pt_xy = all_seaing_perception::projectPCLPtToPixel(m_cam_model, pcloud_ptr->points[ind], m_is_sim);
                cluster_pts.push_back(std::make_pair(cloud_pt_xy, cloud_pt_hsv));
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
        }

        // GO THROUGH ALL THE CLUSTER/CONTOUR PAIRS AND FIND THE BEST ONE BASED ON THE OPTIMALITY METRIC WITH THE WEIGHTS
        long long min_pair_cost = -1;
        int opt_contour_id = -1, opt_cluster_id = -1;
        for (int contour = 0; contour < contours_pts.size(); contour++){
            long long contour_size = contours_pts[contour].size();
            std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> contour_qts = contours_qts[contour];
            
            long long contour_cost = 0;
            for (std::pair<cv::Point2d, std::vector<long long>> contour_pt : contours_pts[contour]){
                if (label_color_map[bbox.label]!="red"){
                    contour_cost += (long long)color_range_penalty(m_contour_detection_color_weights, contour_matching_color_range_map[label_color_map[bbox.label]], contour_pt.second)/contour_size;
                }else{
                    std::vector<int> adjusted_color_range = contour_matching_color_range_map["red"]; //(Hmin,Hmax, ...)
                    adjusted_color_range[1] += 90; // Hmax from red is low, add 90
                    adjusted_color_range[1] = contour_matching_color_range_map["red2"][0]-90; // Hmin from red2 is high, add 90 and subtract 180 -> -90
                    contour_cost += (long long)color_range_penalty(m_contour_detection_color_weights, adjusted_color_range, contour_pt.second)/contour_size;
                }
            }
            for (int cluster = 0; cluster < clusters_pts.size(); cluster++){
                long long cluster_size = clusters_pts[cluster].size();
                std::pair<std::pair<cv::Point2d, std::vector<long long>>, std::pair<cv::Point2d, std::vector<long long>>> cluster_qts = clusters_qts[cluster];
                
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

        pcl::PointCloud<pcl::PointXYZHSV>::Ptr refined_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        refined_cloud_ptr->header = pcloud_ptr->header;
        for (pcl::index_t ind : clusters_indices[opt_cluster_id].indices){
            pcl::PointXYZHSV pt = pcloud_ptr->points[ind];
            refined_cloud_ptr->push_back(pt);
        }
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr refined_cloud_base_link_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        all_seaing_perception::transformPCLCloud(*refined_cloud_ptr, *refined_cloud_base_link_ptr, m_cam_base_link_tf);

        all_seaing_perception::Obstacle<pcl::PointXYZHSV> obstacle(m_local_header, refined_cloud_base_link_ptr, id++, false);
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