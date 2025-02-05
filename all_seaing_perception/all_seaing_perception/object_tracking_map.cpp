#include "all_seaing_perception/object_tracking_map.hpp"

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map"){
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "odom");
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("range_uncertainty", 1.0);
    this->declare_parameter<double>("bearing_uncertainty", 1.0);
    this->declare_parameter<double>("new_object_slam_threshold", 1.0);
    this->declare_parameter<double>("init_new_cov", 10.0);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_range_std = this->get_parameter("range_uncertainty").as_double();
    m_bearing_std = this->get_parameter("bearing_uncertainty").as_double();
    m_new_obj_slam_thres = this->get_parameter("new_object_slam_threshold").as_double();
    m_init_new_cov = this->get_parameter("init_new_cov").as_double();
    RCLCPP_INFO(this->get_logger(), "OBSTACLE SEG THRESHOLD: %lf, DROP THRESHOLD: %lf, SLAM RANGE UNCERTAINTY: %lf, SLAM BEARING UNCERTAINTY: %lf, SLAM NEW OBJECT THRESHOLD: %lf", m_obstacle_seg_thresh, m_obstacle_drop_thresh, m_range_std, m_bearing_std, m_new_obj_slam_thres);
    // Initialize navigation variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_heading = 0;

    // Initialize publishers and subscribers
    m_untracked_map_pub =
        this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/refined_untracked", 10);
    m_tracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_tracked", 10);
    m_map_cov_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "obstacle_map/map_cov_viz", 10);
    m_object_sub = this->create_subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>(
        "refined_object_point_clouds_segments", rclcpp::SensorDataQoS(),
        std::bind(&ObjectTrackingMap::object_track_map_publish, this, std::placeholders::_1));
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&ObjectTrackingMap::odom_callback, this, std::placeholders::_1));
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_topic", 10, std::bind(&ObjectTrackingMap::intrinsics_cb, this, std::placeholders::_1));
}

ObjectCloud::ObjectCloud(rclcpp::Time t, int l, pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc, pcl::PointCloud<pcl::PointXYZHSV>::Ptr glob){
    id = -1;
    label = l;
    time_seen = t;
    last_dead = rclcpp::Time(0);
    time_dead = rclcpp::Duration(0,0);
    is_dead = false;
    global_pcloud_ptr = glob;
    for(pcl::PointXYZHSV &global_pt : global_pcloud_ptr->points){
        global_centroid.x += global_pt.x/((double)global_pcloud_ptr->points.size());
        global_centroid.y += global_pt.y/((double)global_pcloud_ptr->points.size());
        global_centroid.z += global_pt.z/((double)global_pcloud_ptr->points.size());
    }
    this->update_loc_pcloud(loc);
}

void ObjectCloud::update_loc_pcloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc){
    local_pcloud_ptr = loc;
    for(pcl::PointXYZHSV &local_pt : local_pcloud_ptr->points){
        local_centroid.x += local_pt.x/((double)local_pcloud_ptr->points.size());
        local_centroid.y += local_pt.y/((double)local_pcloud_ptr->points.size());
        local_centroid.z += local_pt.z/((double)local_pcloud_ptr->points.size());
    }
}

void ObjectTrackingMap::odom_callback(const nav_msgs::msg::Odometry &msg) {
    m_nav_x = msg.pose.pose.position.x;
    m_nav_y = msg.pose.pose.position.y;
    tf2::Quaternion q;
    q.setW(msg.pose.pose.orientation.w);
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    tf2::Matrix3x3 m(q);
    double r, p, y;
    m.getRPY(r, p, y);
    m_nav_heading = y;

    m_nav_pose = msg.pose.pose;
}

void ObjectTrackingMap::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    RCLCPP_INFO(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

geometry_msgs::msg::TransformStamped ObjectTrackingMap::get_tf(const std::string &in_target_frame,
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

template <typename T>
T ObjectTrackingMap::convert_to_global(double nav_x, double nav_y, double nav_heading, T point) {
    T new_point = point;//to keep color-related data and z coordinate
    double magnitude = std::hypot(point.x, point.y);
    double point_angle = std::atan2(point.y, point.x);
    new_point.x = nav_x + std::cos(nav_heading + point_angle) * magnitude;
    new_point.y = nav_y + std::sin(nav_heading + point_angle) * magnitude;
    return new_point;
}

//TODO: Check it actually works, individually from the other parts
template <typename T>
T ObjectTrackingMap::convert_to_local(double nav_x, double nav_y, double nav_heading, T point) {
    T new_point = point;//to keep color-related data and z coordinate
    double diff_x = point.x - nav_x;
    double diff_y = point.y - nav_y;
    double magnitude = std::hypot(diff_x, diff_y);
    double point_angle = std::atan2(diff_y, diff_x);
    new_point.x = std::cos(point_angle - nav_heading) * magnitude;
    new_point.y = std::sin(point_angle - nav_heading) * magnitude;
    RCLCPP_INFO(this->get_logger(), "GLOBAL: (%lf, %lf) -> LOCAL: (%lf, %lf)", point.x, point.y, new_point.x, new_point.y);
    return new_point;
}

//TODO: Check it works, especially combined with the above
//(range, bearing, signature)
template <typename T>
ObjectTrackingMap::det_rbs ObjectTrackingMap::local_to_range_bearing_signature(T point, int label){
    double range = std::hypot(point.x, point.y);
    double bearing = std::atan2(point.y, point.x);
    RCLCPP_INFO(this->get_logger(), "LOCAL: (%lf, %lf) -> RBS: (%lf, %lf, %d)", point.x, point.y, range, bearing, label);
    return det_rbs(range, bearing, label);
}

template <typename T_matrix>
std::string matrix_to_string(T_matrix matrix){
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

void ObjectTrackingMap::visualize_predictions(){
    visualization_msgs::msg::MarkerArray ellipse_arr;
    for(int i=0; i<m_num_obj; i++){
        Eigen::Vector2f obj_mean = m_map.segment(2*i, 2);//z coord is the label (will visualize the variance of that too to identify possible bugs in its update, because it should not be changed)
        Eigen::Matrix2f obj_cov = m_cov.block(2*i, 2*i, 2, 2);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(obj_cov);
        double a_x = eigen_solver.eigenvalues()(0);
        double a_y = eigen_solver.eigenvalues()(1);
        RCLCPP_INFO(this->get_logger(), "OBJECT %d COVARIANCE AXES LENGTHS: (%lf, %lf)", i, a_x, a_y);
        Eigen::Vector2f axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2f axis_y = eigen_solver.eigenvectors().col(1);
        // double cov_scale = sqrt(5.991);
        double cov_scale = 1;
        tf2::Matrix3x3 rot_mat(
            axis_x(0), axis_y(0), 0,
            axis_x(1), axis_y(1), 0,
            0, 0, 1);
        tf2::Quaternion quat_rot;
        rot_mat.getRotation(quat_rot);
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;
        ellipse.pose.position.x = obj_mean(0);
        ellipse.pose.position.y = obj_mean(1);
        ellipse.pose.position.z = 0;
        ellipse.pose.orientation = tf2::toMsg(quat_rot);
        ellipse.scale.x = cov_scale*sqrt(a_x);
        ellipse.scale.y = cov_scale*sqrt(a_y);
        ellipse.scale.z = 1;
        ellipse.color.a = 1;
        ellipse.color.r = 1;
        ellipse.header.frame_id = m_global_frame_id;
        ellipse.id = i;
        ellipse_arr.markers.push_back(ellipse);
    }
    m_map_cov_viz_pub->publish(ellipse_arr);
}

void ObjectTrackingMap::object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg){
    if(msg->objects.size()==0) return;
    RCLCPP_INFO(this->get_logger(), "GOT DATA");
    std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles;
    for(all_seaing_interfaces::msg::LabeledObjectPointCloud obj : msg->objects){
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(obj.cloud, *local_obj_pcloud);
        for(pcl::PointXYZHSV &pt : local_obj_pcloud->points){
            pcl::PointXYZHSV global_pt = this->convert_to_global(m_nav_x, m_nav_y, m_nav_heading, pt);
            global_obj_pcloud->push_back(global_pt);
        }
        std::shared_ptr<ObjectCloud> obj_cloud (new ObjectCloud(obj.time, obj.label, local_obj_pcloud, global_obj_pcloud));
        detected_obstacles.push_back(obj_cloud);
    }
    // Make and publish untracked map
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> untracked_obs;
    std::vector<int> untracked_labels;
    for(std::shared_ptr<ObjectCloud> det_obs : detected_obstacles){
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(pcl::PointXYZHSV pt : det_obs->local_pcloud_ptr->points){
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            raw_cloud->push_back(i_pt);
        }
        std::vector<int> ind(raw_cloud->size());
        std::iota (std::begin(ind), std::end(ind), 0);
        std::shared_ptr<all_seaing_perception::Obstacle> untracked_ob(
            new all_seaing_perception::Obstacle(raw_cloud, ind, m_obstacle_id++,
                                                det_obs->time_seen, m_nav_x, m_nav_y, m_nav_heading));
        untracked_labels.push_back(det_obs->label);
        untracked_obs.push_back(untracked_ob);
    }
    this->publish_map(msg->objects[0].cloud.header, "untracked", true, untracked_obs, m_untracked_map_pub, untracked_labels);

    // LIDAR -> Camera transform
    // if (!m_pc_cam_tf_ok)
    //     RCLCPP_INFO(this->get_logger(), "CAMERA FRAME ID: %s", msg->objects[0].segment.header.frame_id.c_str());
    //     RCLCPP_INFO(this->get_logger(), "LIDAR FRAME ID: %s", msg->objects[0].cloud.header.frame_id.c_str());
    //     m_pc_cam_tf = get_tf(msg->objects[0].segment.header.frame_id, msg->objects[0].cloud.header.frame_id);

    //TODO: Maybe use some better metric except from only the centroid of the objects

    //EKF SLAM ("Probabilistic Robotics", Seb. Thrun, implementation, removed robot pose from the state and the respective motion updates, changed assignment algorithm)
    Eigen::Matrix<float, 2, 2> Q{
        {m_range_std, 0},
        {0, m_bearing_std},
    };
    std::vector<std::vector<float>> p;
    for(std::shared_ptr<ObjectCloud> det_obs : detected_obstacles){
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = local_to_range_bearing_signature(det_obs->local_centroid, det_obs->label);
        p.push_back(std::vector<float>());
        for(int tracked_id = 0; tracked_id < m_num_obj; tracked_id++){
            float d_x = m_map(2*tracked_id) - m_nav_x;
            float d_y = m_map(2*tracked_id+1) - m_nav_y;
            float q = d_x*d_x + d_y*d_y;
            Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
            Eigen::MatrixXf F = Eigen::MatrixXf::Zero(2, 2*m_num_obj);
            F.block(0, 2*tracked_id, 2, 2) = Eigen::MatrixXf::Identity(2,2);
            Eigen::Matrix<float, 2, 2> h{
                { std::sqrt(q)*d_x, std::sqrt(q)*d_y},
                { -d_y, d_x},
            };
            //Do not store vectors, since they don't compute covariance with other obstacles in the same detection batch
            Eigen::MatrixXf H = h*F/q;
            Eigen::MatrixXf Psi = H*m_cov*H.transpose()+Q;
            Eigen::Vector2f z_actual(range, bearing);
            
            p.back().push_back((z_actual-z_pred).transpose()*Psi.inverse()*(z_actual-z_pred));
        }
    }

    //Assign each detection to a tracked or new object using the computed probabilities (actually -logs?)
    std::vector<int> match(detected_obstacles.size(), -1);
    float min_p = 0;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    if(!p.empty()) RCLCPP_INFO(this->get_logger(), "P SIZE: (%d, %d), DETECTED OBSTACLES: %d, TRACKED OBSTACLES: %d", p.size(), p.back().size(), detected_obstacles.size(), m_num_obj);
    while(min_p < m_new_obj_slam_thres){
        min_p = m_new_obj_slam_thres;
        std::pair<int,int> best_match = std::make_pair(-1,-1);
        for(int i=0; i < detected_obstacles.size(); i++){
            if(chosen_detected.count(i)) continue;
            for(int tracked_id = 0; tracked_id < m_num_obj; tracked_id++){
                if(chosen_tracked.count(tracked_id) || m_tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label) continue;
                RCLCPP_INFO(this->get_logger(), "P(%d, %d)=%lf", i, tracked_id, p[i][tracked_id]);
                if (p[i][tracked_id] < min_p) {
                    best_match = std::make_pair(i, tracked_id);
                    min_p = p[i][tracked_id];
                }
            }
        }
        if(min_p < m_new_obj_slam_thres){
            RCLCPP_INFO(this->get_logger(), "MATCHING (%d, %d)", best_match.first, best_match.second);
            match[best_match.first] = best_match.second;
            chosen_tracked.insert(best_match.second);
            chosen_detected.insert(best_match.first);
        }
    }


    //Update vectors, now with known correspondence
    //TODO: Check for float overflows (maybe due to the infs in new obstacle covariances) and find out how to prevent them
    for(int i=0; i < detected_obstacles.size(); i++){
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = local_to_range_bearing_signature(detected_obstacles[i]->local_centroid, detected_obstacles[i]->label);
        if(match[i] == -1){
            //increase object count and expand & initialize matrices
            m_num_obj++;
            // RCLCPP_INFO(this->get_logger(), "MAP BEFORE RESIZE");
            // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_map).c_str());
            m_map.conservativeResize(2*m_num_obj);
            m_map.tail(2) = Eigen::Vector2f(m_nav_x, m_nav_y) + range*Eigen::Vector2f(std::cos(bearing + m_nav_heading), std::sin(bearing + m_nav_heading));
            // RCLCPP_INFO(this->get_logger(), "MAP AFTER RESIZE");
            // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_map).c_str());
            // RCLCPP_INFO(this->get_logger(), "COVARIANCE BEFORE RESIZE");
            // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_cov).c_str());
            m_cov.conservativeResizeLike(Eigen::MatrixXf::Zero(2*m_num_obj, 2*m_num_obj)); 
            RCLCPP_INFO(this->get_logger(), "INITIAL NEW COVARIANCE = %lf", m_init_new_cov);
            //TODO: Replace that initialization part with directly computing the covariance, to not have to deal with infinites
            Eigen::Matrix<float, 2, 2> init_new_cov{
                {m_init_new_cov, 0},
                {0, m_init_new_cov},
            };
            m_cov.bottomRightCorner(2,2) = init_new_cov;
            // RCLCPP_INFO(this->get_logger(), "COVARIANCE AFTER RESIZE");
            // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_cov).c_str());
            //add object to tracked obstacles vector
            detected_obstacles[i]->id = m_obstacle_id++;
            m_tracked_obstacles.push_back(detected_obstacles[i]);
        }
        int tracked_id = match[i]>0?match[i]:m_num_obj-1;
        float d_x = m_map(2*tracked_id) - m_nav_x;
        float d_y = m_map(2*tracked_id+1) - m_nav_y;
        float q = d_x*d_x + d_y*d_y;
        Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
        Eigen::MatrixXf F = Eigen::MatrixXf::Zero(2, 2*m_num_obj);
        F.block(0, 2*tracked_id, 2, 2) = Eigen::MatrixXf::Identity(2,2);
        Eigen::Matrix<float, 2, 2> h{
            { std::sqrt(q)*d_x, std::sqrt(q)*d_y},
            { -d_y, d_x},
        };
        // RCLCPP_INFO(this->get_logger(), "h");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(h).c_str());
        // RCLCPP_INFO(this->get_logger(), "F");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(F).c_str());
        // Eigen::MatrixXf H = h*F/q;
        // RCLCPP_INFO(this->get_logger(), "H");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(H).c_str());
        // RCLCPP_INFO(this->get_logger(), "q: %lf", q);
        // RCLCPP_INFO(this->get_logger(), "Q");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(Q).c_str());
        // Eigen::MatrixXf K = m_cov*H.transpose()*(H*m_cov*H.transpose()+Q).inverse();
        // RCLCPP_INFO(this->get_logger(), "K");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(K).c_str());
        Eigen::Vector2f z_actual(range, bearing);
        // RCLCPP_INFO(this->get_logger(), "MAP BEFORE UPDATE");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_map).c_str());
        // RCLCPP_INFO(this->get_logger(), "COVARIANCE BEFORE UPDATE");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(m_cov).c_str());
        // RCLCPP_INFO(this->get_logger(), "z_pred");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(z_pred).c_str());
        // RCLCPP_INFO(this->get_logger(), "z_actual");
        // RCLCPP_INFO(this->get_logger(), matrix_to_string(z_actual).c_str());
        m_map += K*(z_actual-z_pred);
        m_cov = (Eigen::MatrixXf::Identity(2*m_num_obj, 2*m_num_obj)-K*H)*m_cov;
        RCLCPP_INFO(this->get_logger(), "MAP AFTER UPDATE");
        RCLCPP_INFO(this->get_logger(), matrix_to_string(m_map).c_str());
        RCLCPP_INFO(this->get_logger(), "COVARIANCE AFTER UPDATE");
        RCLCPP_INFO(this->get_logger(), matrix_to_string(m_cov).c_str());

        //update data for matched obstacles (we'll update position after we update SLAM with all points)
        detected_obstacles[i]->id = m_tracked_obstacles[tracked_id]->id;
        m_tracked_obstacles[tracked_id] = detected_obstacles[i];
    }

    // Update all (new and old, since they also have correlation with each other) objects global positions (including the cloud, and then the local points respectively)
    for(int i=0; i<m_tracked_obstacles.size(); i++){
        pcl::PointXYZ upd_glob_centr = m_tracked_obstacles[i]->global_centroid;
        upd_glob_centr.x = m_map[2*i];
        upd_glob_centr.y = m_map[2*i+1];//to not lose the z coordinate, it's useful for checking if the objects should be in the camera frame
        pcl::PointXYZ upd_loc_centr;
        upd_loc_centr = this->convert_to_local(m_nav_x, m_nav_y, m_nav_heading, upd_glob_centr);
        RCLCPP_INFO(this->get_logger(), "ROBOT POSE: (%lf, %lf, %lf), TRACKED OBSTACLE %d GLOBAL COORDS: (%lf, %lf), LOCAL COORDS: (%lf, %lf)", m_nav_x, m_nav_y, m_nav_heading, m_tracked_obstacles[i]->id, upd_glob_centr.x, upd_glob_centr.y, upd_loc_centr.x, upd_loc_centr.y);
        //move everything based on the difference between that and the previous assumed global (then local) centroid
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        for(pcl::PointXYZHSV& global_pt : m_tracked_obstacles[i]->global_pcloud_ptr->points){
            global_pt.x+=upd_glob_centr.x-m_tracked_obstacles[i]->global_centroid.x;
            global_pt.y+=upd_glob_centr.y-m_tracked_obstacles[i]->global_centroid.y;
            upd_local_obj_pcloud->push_back(this->convert_to_local(m_nav_x, m_nav_y, m_nav_heading, global_pt));
        }
        m_tracked_obstacles[i]->local_pcloud_ptr = upd_local_obj_pcloud;
        m_tracked_obstacles[i]->global_centroid = upd_glob_centr;
        m_tracked_obstacles[i]->local_centroid = upd_loc_centr;
    }

    // Filter old obstacles
    for (int tracked_id = 0; tracked_id < m_tracked_obstacles.size(); tracked_id++) {
        if(chosen_tracked.count(tracked_id)) continue;
        // Check if in FoV (which'll mean it's dead, at least temporarily)
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = m_tracked_obstacles[tracked_id]->local_centroid.x;
        lidar_point.y = m_tracked_obstacles[tracked_id]->local_centroid.y;
        lidar_point.z = m_tracked_obstacles[tracked_id]->local_centroid.z;
        geometry_msgs::msg::Point camera_point = lidar_point;//ALREADY IN THE SAME FRAME, WAS TRANSFORMED BEFORE BEING PUBLISHED BY bbox_project_pcloud.cpp
        // tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, camera_point, m_pc_cam_tf);
        cv::Point2d xy_rect = m_cam_model.project3dToPixel(
            cv::Point3d(camera_point.y, camera_point.z, -camera_point.x));
        RCLCPP_INFO(this->get_logger(), "OBSTACLE ID %d (%lf, %lf, %lf)->(%lf, %lf)", m_tracked_obstacles[tracked_id]->id, lidar_point.x, lidar_point.y, lidar_point.z, xy_rect.x, xy_rect.y);
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
            (xy_rect.y < m_cam_model.cameraInfo().height) && (lidar_point.x >= 0)) {
            // Dead
            if(m_tracked_obstacles[tracked_id]->is_dead){
                // Was also dead before, add time dead
                RCLCPP_INFO(this->get_logger(), "OBSTACLE ID %d TIME PERIOD FROM PREVIOUS DEAD: %lf - %lf", m_tracked_obstacles[tracked_id]->id, m_tracked_obstacles[tracked_id]->last_dead.seconds(), rclcpp::Time(msg->objects[0].time).seconds());
                m_tracked_obstacles[tracked_id]->time_dead = rclcpp::Time(msg->objects[0].time) - m_tracked_obstacles[tracked_id]->last_dead + m_tracked_obstacles[tracked_id]->time_dead;
                RCLCPP_INFO(this->get_logger(), "OBSTACLE ID %d DEAD FOR %lf SECONDS, OBSTACLE DROP THRESHOLD: %lf", m_tracked_obstacles[tracked_id]->id, m_tracked_obstacles[tracked_id]->time_dead.seconds(), m_obstacle_drop_thresh);
                if(m_tracked_obstacles[tracked_id]->time_dead.seconds() > m_obstacle_drop_thresh){
                    RCLCPP_INFO(this->get_logger(), "OBSTACLE ID %d DROPPED", m_tracked_obstacles[tracked_id]->id);
                    m_tracked_obstacles.erase(m_tracked_obstacles.begin()+tracked_id);
                    //need to be careful with deleting the object from the SLAM map mean & covariance matrices
                    std::vector<int> new_ind(2*m_num_obj-2);
                    std::iota(std::begin(new_ind), std::begin(new_ind)+2*tracked_id, 0);
                    std::iota(std::begin(new_ind)+2*tracked_id+2, std::end(new_ind), 2*tracked_id+2);
                    m_map = m_map(new_ind);
                    m_cov = m_map(new_ind, new_ind);
                    m_num_obj--;
                    tracked_id--;
                    continue;
                }
            }
            m_tracked_obstacles[tracked_id]->is_dead = true;
            m_tracked_obstacles[tracked_id]->last_dead = msg->objects[0].time;
        }else{
            m_tracked_obstacles[tracked_id]->is_dead = false;
        }
    }

    // Publish map with tracked obstacles
    //TODO: Find out the cause and solve convex hull computation errors
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> tracked_obs;
    std::vector<int> tracked_labels;
    for(std::shared_ptr<ObjectCloud> t_ob : m_tracked_obstacles){
        pcl::PointCloud<pcl::PointXYZI>::Ptr tracked_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(pcl::PointXYZHSV pt : t_ob->local_pcloud_ptr->points){
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            tracked_cloud->push_back(i_pt);
        }
        std::vector<int> ind(tracked_cloud->size());
        std::iota (std::begin(ind), std::end(ind), 0);
        std::shared_ptr<all_seaing_perception::Obstacle> tracked_ob(
            new all_seaing_perception::Obstacle(tracked_cloud, ind, t_ob->id,
                                                t_ob->time_seen, m_nav_x, m_nav_y, m_nav_heading));
        tracked_labels.push_back(t_ob->label);
        tracked_obs.push_back(tracked_ob);
    }
    this->publish_map(msg->objects[0].cloud.header, "tracked", true, tracked_obs, m_tracked_map_pub, tracked_labels);
    RCLCPP_INFO(this->get_logger(), "AFTER TRACKED MAP PUBLISHING");
    //publish covariance ellipsoid markers to understand prediction uncertainty (& fine-tune & debug)
    //TODO: Fine-tune the object matching threshold (how many sigma it should be far from the other to be considered new)
    this->visualize_predictions();
}

void ObjectTrackingMap::publish_map(
    std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
    const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub, std::vector<int> labels) {

    // Create global header
    std_msgs::msg::Header global_header = std_msgs::msg::Header();
    global_header.frame_id = m_global_frame_id;
    global_header.stamp = local_header.stamp;

    all_seaing_interfaces::msg::ObstacleMap map_msg;
    map_msg.ns = ns;
    map_msg.local_header = local_header;
    map_msg.header = global_header;
    map_msg.is_labeled = is_labeled;
    for (unsigned int i = 0; i < map.size(); i++) {
        all_seaing_interfaces::msg::Obstacle det_obstacle;
        map[i]->to_ros_msg(local_header, global_header, det_obstacle);
        if(is_labeled){
            det_obstacle.label = labels[i];
        }
        map_msg.obstacles.push_back(det_obstacle);
    }
    map_msg.pose = m_nav_pose;
    pub->publish(map_msg);
}

ObjectTrackingMap::~ObjectTrackingMap() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMap>());
    rclcpp::shutdown();
    return 0;
}