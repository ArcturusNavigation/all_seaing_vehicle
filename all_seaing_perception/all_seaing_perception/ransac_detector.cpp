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

visualization_msgs::msg::MarkerArray RANSACDetector::visualize_plane(all_seaing_interfaces::msg::LabeledObjectPlane msg, int& marker_id){
    visualization_msgs::msg::MarkerArray marker_arr;

    visualization_msgs::msg::Marker normal_marker;
    normal_marker.type = visualization_msgs::msg::Marker::ARROW;
    normal_marker.pose = msg.normal_ctr;
    normal_marker.scale.x = 1;
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
    plane_marker.color.b = 1;
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
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id).markers);
    }
    for (auto plane : msg.coplanar_merged){
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id).markers);
    }
    for (auto plane : msg.coplanar_indiv){
        vector_extend(marker_arr.markers, visualize_plane(plane, marker_id).markers);
    }
    return marker_arr;
}

void RANSACDetector::object_pcl_cb(
    const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &in_pcl_msg){

    for (all_seaing_interfaces::msg::LabeledObjectPointCloud labeled_pcl : in_pcl_msg->objects){
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(labeled_pcl.cloud, *in_cloud_ptr);
        m_labeled_pcls.push_back(std::make_pair(labeled_pcl.label, in_cloud_ptr));
    }

    m_header = in_pcl_msg->header;

    auto object_planes = all_seaing_interfaces::msg::LabeledObjectPlaneArray();
    object_planes.header = m_header;

    // Deal with individual banners
    for (auto labeled_pcl : m_labeled_pcls){
        int label = labeled_pcl.first;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;
        if (m_labels.count(m_id_label_map[label])){
            // get centroid, normal, and size, and add to object_planes.objects
            Eigen::Vector3d centroid, size;
            Eigen::Matrix3d normal;
            std::tie(centroid, normal, size) = all_seaing_perception::PCLRANSAC(*pcl_ptr, m_dist_thres, m_max_iters);
            object_planes.objects.push_back(to_plane_msg(label, centroid, normal, size));
        }
        // check if in a coplanar set
        if (m_coplanar_id.count(m_id_label_map[label])){
            for (int merge_id : m_coplanar_id[m_id_label_map[label]]){
                m_coplanar_pcls[merge_id].push_back(labeled_pcl);
            }
        }
    }

    // Deal with coplanar sets
    for(auto iter = m_coplanar_pcls.begin(); iter != m_coplanar_pcls.end(); ++iter)
    {
        int id_merge = iter->first;
        auto labeled_pcls = iter->second;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr merged_pcloud_ptr(new pcl::PointCloud<pcl::PointXYZHSV>);
        for (auto labeled_pcl : labeled_pcls){
            int label = labeled_pcl.first;
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;
            *merged_pcloud_ptr += *pcl_ptr;
        }

        // get centroid, normal, and size, and add to object_planes.coplanar_merged
        Eigen::Vector3d merged_ctr, merged_size;
        Eigen::Matrix3d merged_normal;
        std::tie(merged_ctr, merged_normal, merged_size) = all_seaing_perception::PCLRANSAC(*merged_pcloud_ptr, m_dist_thres, m_max_iters);
        object_planes.coplanar_merged.push_back(to_plane_msg(id_merge, merged_ctr, merged_normal, merged_size));

        // get indiv refined planes
        for (auto labeled_pcl : labeled_pcls){
            int label = labeled_pcl.first;
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_ptr = labeled_pcl.second;

            // TODO: Project the inliers wrt the merged frame, remove outliers for the resulting point cloud, and add refined frame to object_planes.coplanar_indiv
            // Can probably use some of the helper functions for RANSAC in perception_utilities, do that
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
}

RANSACDetector::~RANSACDetector() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RANSACDetector>());
    rclcpp::shutdown();
    return 0;
}