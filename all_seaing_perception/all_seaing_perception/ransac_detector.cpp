#include "all_seaing_perception/ransac_detector.hpp"

RANSACDetector::RANSACDetector() : Node("ransac_detector"){
    // Parameters
    this->declare_parameter("ransac_params_file", "");
    m_ransac_params_file = this->get_parameter("ransac_params_file").as_string();

    this->declare_parameter("label_mappings_file", "");
    m_label_mappings_file = this->get_parameter("label_mappings_file").as_string();

    // Subscriptions
    m_object_pcl_sub = this->create_subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>(
        "labeled_object_point_clouds", 10, std::bind(&RANSACDetector::object_pcl_cb, this, std::placeholders::_1));

    // Publishers
    m_ransac_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("object_planes_viz", 5);
    m_object_ransac_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPlaneArray>("object_planes", 5);

    // get cluster-contour matching & selection parameters from yaml
    std::ifstream ransac_param_yaml(m_ransac_params_file);
    if (ransac_param_yaml.is_open()) {
        m_ransac_params_config_yaml = YAML::Load(ransac_param_yaml);

        std::vector<std::string> labels = m_ransac_params_config_yaml["labels"].as<std::vector<std::string>>();
        m_labels = std::set<std::string>(labels.begin(), labels.end());

        YAML::Node coplanar = m_ransac_params_config_yaml["coplanar"];
        for (YAML::const_iterator it = coplanar.begin(); it != coplanar.end(); ++it) {
            std::vector<std::string> coplanars = it->second.as<std::vector<std::string>>();
            for (std::string coplanar_label : coplanars){
                m_coplanar_id[coplanar_label].push_back(it->first.as<int>());
            }
        }

        m_max_iters = m_ransac_params_config_yaml["max_iters"].as<int>();
        m_dist_thres = m_ransac_params_config_yaml["dist_thres"].as<double>();
        m_min_inliers = m_ransac_params_config_yaml["min_inliers"].as<int>();
        m_clust_dist = m_ransac_params_config_yaml["clust_dist"].as<double>();
        m_plane_angle_thres = m_ransac_params_config_yaml["plane_angle_thres"].as<double>();
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", m_ransac_params_file.c_str());
    }
    ransac_param_yaml.close();
    
    // get label mappings from yaml
    std::ifstream label_yaml(m_label_mappings_file);
    if (label_yaml.is_open()) {
        m_label_mappings_config_yaml = YAML::Load(label_yaml);  
        for (YAML::const_iterator it = m_label_mappings_config_yaml.begin(); it != m_label_mappings_config_yaml.end(); ++it) {
            m_id_label_map[it->second.as<int>()] = it->first.as<std::string>();
            m_label_id_map[it->first.as<std::string>()] = it->second.as<int>();
            RCLCPP_INFO(this->get_logger(), "%s -> %d", it->first.as<std::string>().c_str(), it->second.as<int>());
        }
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file: %s", m_label_mappings_file.c_str());
    }
    label_yaml.close();
}

all_seaing_interfaces::msg::LabeledObjectPlane RANSACDetector::to_plane_msg(int label, Eigen::Vector3d centroid, Eigen::Matrix3d normal, Eigen::Vector3d size){
    auto object_plane = all_seaing_interfaces::msg::LabeledObjectPlane();
    object_plane.label = label;
    object_plane.normal_ctr.position.x = centroid[0];
    object_plane.normal_ctr.position.y = centroid[1];
    object_plane.normal_ctr.position.z = centroid[2];

    tf2::Quaternion quat;
    tf2::Matrix3x3 mat(
        normal(0,0), normal(0,1), normal(0,2),
        normal(1,0), normal(1,1), normal(1,2),
        normal(2,0), normal(2,1), normal(2,2)
    );
    mat.getRotation(quat);
    object_plane.normal_ctr.orientation = tf2::toMsg(quat);

    object_plane.size.x = 0.01;
    object_plane.size.y = size[1];
    object_plane.size.z = size[2];

    return object_plane;
}

visualization_msgs::msg::MarkerArray RANSACDetector::visualize_plane(all_seaing_interfaces::msg::LabeledObjectPlane msg, int& marker_id, double r, double g, double b){
    visualization_msgs::msg::MarkerArray marker_arr;

    visualization_msgs::msg::Marker normal_marker;
    normal_marker.type = visualization_msgs::msg::Marker::ARROW;
    normal_marker.pose = msg.normal_ctr;
    normal_marker.scale.x = 0.2;
    normal_marker.scale.y = 0.01;
    normal_marker.scale.z = 0.01;
    normal_marker.color.a = 1;
    normal_marker.color.r = 1;
    normal_marker.header = m_header;
    normal_marker.id = marker_id++;
    marker_arr.markers.push_back(normal_marker);

    visualization_msgs::msg::Marker plane_marker;
    plane_marker.type = visualization_msgs::msg::Marker::CUBE;
    plane_marker.pose = msg.normal_ctr;
    plane_marker.scale.x = 0.01;
    plane_marker.scale.y = msg.size.y;
    plane_marker.scale.z = msg.size.z;
    plane_marker.color.a = 1;
    plane_marker.color.r = r;
    plane_marker.color.g = g;
    plane_marker.color.b = b;
    plane_marker.header = m_header;
    plane_marker.id = marker_id++;
    marker_arr.markers.push_back(plane_marker);
    
    return marker_arr;
}

template<typename type_t>
void vector_extend(typename std::vector<type_t> &v1, const typename std::vector<type_t> &v2){
    v1.insert(v1.end(), v2.begin(), v2.end());
}

visualization_msgs::msg::MarkerArray RANSACDetector::visualize_planes(all_seaing_interfaces::msg::LabeledObjectPlaneArray msg){
    int marker_id = 0;
    visualization_msgs::msg::MarkerArray marker_arr;
    for (auto plane : msg.objects){
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id, 0, 0, 1).markers);
    }
    for (auto plane : msg.coplanar_merged){
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id, 0, 1, 0).markers);
    }
    for (auto plane : msg.coplanar_indiv){
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id, 1, 0, 0).markers);
    }
    return marker_arr;
}

void RANSACDetector::object_pcl_cb(
    const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &in_pcl_msg){

    std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>> labeled_pcls; // (id, pcl)
    
    for (all_seaing_interfaces::msg::LabeledObjectPointCloud labeled_pcl : in_pcl_msg->objects){
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(labeled_pcl.cloud, *in_cloud_ptr);
        labeled_pcls.push_back(std::make_pair(labeled_pcl.label, in_cloud_ptr));
    }

    m_header = in_pcl_msg->header;


    auto object_planes = all_seaing_interfaces::msg::LabeledObjectPlaneArray();
    object_planes.header = m_header;
    std::map<int, std::vector<std::pair<int, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>>> coplanar_pcls; // (id_merge, [(id, pcl), ...])
    // Deal with individual banners
    for (auto labeled_pcl : labeled_pcls){
        int label = labeled_pcl.first;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;
        if (m_labels.count(m_id_label_map[label])){
            // get centroid, normal, and size, and add to object_planes.objects
            Eigen::Vector3d centroid, size;
            Eigen::Matrix3d normal;
            int num_inliers;
            if (pcl_ptr->size() < 3) continue;
            std::tie(centroid, normal, size, num_inliers) = all_seaing_perception::PCLRANSAC(*pcl_ptr, m_dist_thres, m_max_iters, true, m_clust_dist);
            if (num_inliers >= m_min_inliers && std::asin(std::abs(normal(2,0)/normal.col(0).norm())) <= m_plane_angle_thres*M_PI/((float)180)){
                object_planes.objects.push_back(to_plane_msg(label, centroid, normal, size));
            }
        }
        // check if in a coplanar set
        if (m_coplanar_id.count(m_id_label_map[label])){
            for (int merge_id : m_coplanar_id[m_id_label_map[label]]){
                coplanar_pcls[merge_id].push_back(labeled_pcl);
            }
        }
    }

    // Deal with coplanar sets
    for(auto iter = coplanar_pcls.begin(); iter != coplanar_pcls.end(); ++iter)
    {
        int id_merge = iter->first;
        auto labeled_pcls = iter->second;
        std::set<std::pair<int, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>> active_pcls(labeled_pcls.begin(), labeled_pcls.end());
        while(!active_pcls.empty()){
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr merged_pcloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
            for (auto labeled_pcl : active_pcls){
                int label = labeled_pcl.first;
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;
                *merged_pcloud_ptr += *pcl_ptr;
            }

            // get centroid, normal, and size, and add to object_planes.coplanar_merged
            Eigen::Vector3d merged_ctr, merged_size;
            Eigen::Matrix3d merged_normal;
            int merged_num_inliers;
            if (merged_pcloud_ptr->size() < 3) continue;
            // TODO: Do the clustering if we only have one banner with inliers, after doing the merged RANSAC and getting each banner's inliers for it
            std::tie(merged_ctr, merged_normal, merged_size, merged_num_inliers) = all_seaing_perception::PCLRANSAC(*merged_pcloud_ptr, m_dist_thres, m_max_iters, (active_pcls.size() == 1)?true:false, (active_pcls.size() == 1)?m_clust_dist:0.0f);
            // TODO: Change this to a threshold larger than m_min_inliers if we see that we have many misdetections
            if (merged_num_inliers < m_min_inliers || std::asin(std::abs(merged_normal(2,0)/merged_normal.col(0).norm())) > m_plane_angle_thres*M_PI/((float)180)) continue;
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr refined_merged_pcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

            // get indiv refined planes
            std::vector<std::set<std::pair<int, pcl::PointCloud<pcl::PointXYZHSV>::Ptr>>::iterator> remove_list;
            for (auto iter = active_pcls.begin(); iter != active_pcls.end(); iter++){
                auto labeled_pcl = *iter;
                int label = labeled_pcl.first;
                pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;

                // Project the inliers wrt the merged frame, remove outliers for the resulting point cloud, and add refined frame to object_planes.coplanar_indiv
                // Can probably use some of the helper functions for RANSAC in perception_utilities, do that
                if (pcl_ptr->size() == 0) continue;
                pcl::PointCloud<pcl::PointXYZ> pcl_inliers_all = all_seaing_perception::pickInliers(*pcl_ptr, merged_ctr, merged_normal, m_dist_thres);
                if (pcl_inliers_all.size() == 0) continue;
                pcl::PointCloud<pcl::PointXYZ> pcl_inliers = all_seaing_perception::pickLargestCluster(all_seaing_perception::pickInliers(*pcl_ptr, merged_ctr, merged_normal, m_dist_thres), m_clust_dist);
                if (pcl_inliers.size() < 2) continue; // TODO: Change this to m_min_inliers or some other threshold if we see that we have many misdetections
                Eigen::Vector3d indiv_ctr, indiv_size;
                std::tie(indiv_ctr, indiv_size) = all_seaing_perception::centroidDims(pcl_inliers, merged_normal);
                object_planes.coplanar_indiv.push_back(to_plane_msg(label, indiv_ctr, merged_normal, indiv_size));
                
                *refined_merged_pcloud_ptr += pcl_inliers;
                
                remove_list.push_back(iter);
            }
            if (refined_merged_pcloud_ptr->size() < 2) continue;
            // Compute centroid & size of merged refined pcloud
            std::tie(merged_ctr, merged_size) = all_seaing_perception::centroidDims(*refined_merged_pcloud_ptr, merged_normal);
            object_planes.coplanar_merged.push_back(to_plane_msg(id_merge, merged_ctr, merged_normal, merged_size));

            // Remove banners already included in a plane
            if (remove_list.empty()) break; // didn't add a new plane, which means there isn't any, remove
            for (auto iter : remove_list){
                active_pcls.erase(iter);
            }
        }
    }
    
    // Visualize RANSAC output

    // delete previous markers
    visualization_msgs::msg::MarkerArray delete_arr;
    visualization_msgs::msg::Marker delete_mark;
    delete_mark.id = 0;
    delete_mark.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_arr.markers.push_back(delete_mark);
    m_ransac_viz_pub->publish(delete_arr);
    // publish new ones
    m_ransac_viz_pub->publish(visualize_planes(object_planes));
    m_object_ransac_pub->publish(object_planes);
}

RANSACDetector::~RANSACDetector() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RANSACDetector>());
    rclcpp::shutdown();
    return 0;
}