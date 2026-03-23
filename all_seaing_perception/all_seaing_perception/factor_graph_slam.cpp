#include "all_seaing_perception/factor_graph_slam.hpp"

FactorGraphSLAM::FactorGraphSLAM() : Node("factor_graph_slam") {
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("slam_frame_id", "slam_map");
    this->declare_parameter<std::string>("local_frame_id", "base_link");
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("range_uncertainty", 1.0);
    this->declare_parameter<double>("bearing_uncertainty", 1.0);
    this->declare_parameter<double>("banner_range_uncertainty", 1.0);
    this->declare_parameter<double>("banner_bearing_uncertainty", 1.0);
    this->declare_parameter<double>("banner_orientation_uncertainty", 1.0);
    this->declare_parameter<double>("motion_gps_xy_noise", 1.0);
    this->declare_parameter<double>("motion_gps_theta_noise", 1.0);
    this->declare_parameter<double>("motion_imu_xy_noise", 1.0);
    this->declare_parameter<double>("motion_imu_theta_noise", 1.0);
    this->declare_parameter<double>("update_gps_xy_uncertainty", 1.0);
    this->declare_parameter<double>("update_odom_theta_uncertainty", 1.0);
    this->declare_parameter<double>("new_object_slam_threshold", 1.0);
    this->declare_parameter<double>("new_banner_slam_threshold", 1.0);
    this->declare_parameter<double>("init_xy_noise", 1.0);
    this->declare_parameter<double>("init_theta_noise", 1.0);
    this->declare_parameter<bool>("track_robot", true);
    this->declare_parameter<bool>("imu_predict", true);
    this->declare_parameter<bool>("gps_update", true);
    this->declare_parameter<double>("normalize_drop_dist", 1.0);
    this->declare_parameter<double>("odom_refresh_rate", 40);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_slam_frame_id = this->get_parameter("slam_frame_id").as_string();
    m_local_frame_id = this->get_parameter("local_frame_id").as_string();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_range_std = this->get_parameter("range_uncertainty").as_double();
    m_bearing_std = this->get_parameter("bearing_uncertainty").as_double();
    m_banner_range_std = this->get_parameter("banner_range_uncertainty").as_double();
    m_banner_bearing_std = this->get_parameter("banner_bearing_uncertainty").as_double();
    m_banner_phi_std = this->get_parameter("banner_orientation_uncertainty").as_double();
    m_gps_xy_noise = this->get_parameter("motion_gps_xy_noise").as_double();
    m_gps_theta_noise = this->get_parameter("motion_gps_theta_noise").as_double();
    m_imu_xy_noise = this->get_parameter("motion_imu_xy_noise").as_double();
    m_imu_theta_noise = this->get_parameter("motion_imu_theta_noise").as_double();
    m_update_gps_xy_uncertainty = this->get_parameter("update_gps_xy_uncertainty").as_double();
    m_update_odom_theta_uncertainty = this->get_parameter("update_odom_theta_uncertainty").as_double();
    m_new_obj_slam_thres = this->get_parameter("new_object_slam_threshold").as_double();
    m_new_banner_slam_thres = this->get_parameter("new_banner_slam_threshold").as_double();
    m_init_xy_noise = this->get_parameter("init_xy_noise").as_double();
    m_init_theta_noise = this->get_parameter("init_theta_noise").as_double();
    m_gps_update = this->get_parameter("gps_update").as_bool();
    
    m_normalize_drop_dist = this->get_parameter("normalize_drop_dist").as_double();
    m_odom_refresh_rate = this->get_parameter("odom_refresh_rate").as_double();

    this->declare_parameter<bool>("direct_tf", true);
    m_direct_tf = this->get_parameter("direct_tf").as_bool();

    this->declare_parameter<bool>("rotate_odom", false);
    m_rotate_odom = this->get_parameter("rotate_odom").as_bool();

    this->declare_parameter<double>("duplicate_thresh", 10.0);
    m_duplicate_thresh = this->get_parameter("duplicate_thresh").as_double();

    this->declare_parameter<bool>("drop_ignore_unlabeled", false);
    m_drop_ignore_unlabeled = this->get_parameter("drop_ignore_unlabeled").as_bool();

    this->declare_parameter<bool>("normalize_drop_thresh", false);
    m_normalize_drop_thresh = this->get_parameter("normalize_drop_thresh").as_bool();

    this->declare_parameter<double>("range_drop_thresh", 10.0);
    m_range_drop_thresh = this->get_parameter("range_drop_thresh").as_double();

    this->declare_parameter<bool>("include_odom_theta", true);
    m_include_odom_theta = this->get_parameter("include_odom_theta").as_bool();

    this->declare_parameter<bool>("include_odom_only_theta", true);
    m_include_odom_only_theta = this->get_parameter("include_odom_only_theta").as_bool();

    RCLCPP_INFO(this->get_logger(), m_include_odom_only_theta ? "GPS: OFF" : "GPS: ON");

    this->declare_parameter<std::string>("data_association", "greedy_exclusive");
    m_data_association_algo = this->get_parameter("data_association").as_string();

    this->declare_parameter<double>("trace_time", 5.0);
    m_trace_time = this->get_parameter("trace_time").as_double();

    this->declare_parameter<bool>("include_unlabeled", false);
    m_include_unlabeled = this->get_parameter("include_unlabeled").as_bool();

    this->declare_parameter<double>("unlabeled_association_threshold", 0.2);
    m_unlabeled_assoc_threshold = this->get_parameter("unlabeled_association_threshold").as_double();

    this->declare_parameter<bool>("track_banners", false);
    m_track_banners = this->get_parameter("track_banners").as_bool();

    RCLCPP_INFO(this->get_logger(), m_track_banners ? "TRACK BANNERS: ON" : "TRACK BANNERS: OFF");

    this->declare_parameter<bool>("diff_position_odom", true);
    m_diff_position_odom = this->get_parameter("diff_position_odom").as_bool();

    this->declare_parameter<int>("odom_queue_size", 10);
    m_odom_queue_size = this->get_parameter("odom_queue_size").as_int();

    this->declare_parameter<int>("detection_queue_size", 10);
    m_detection_queue_size = this->get_parameter("detection_queue_size").as_int();

    this->declare_parameter<double>("odom_detection_timeout", 0.1);
    m_odom_detection_timeout = this->get_parameter("odom_detection_timeout").as_double();

    this->declare_parameter<bool>("gps_based_predictions", true);
    m_gps_based_predictions = this->get_parameter("gps_based_predictions").as_bool();

    this->declare_parameter<bool>("match_numbers_indicators", false);
    m_match_numbers_indicators = this->get_parameter("match_numbers_indicators").as_bool();

    this->declare_parameter<std::vector<std::string>>("banner_names", std::vector<std::string>());
    std::vector<std::string> banner_names = this->get_parameter("banner_names").as_string_array();

    this->declare_parameter<std::vector<int64_t>>("banner_labels", std::vector<int64_t>());
    std::vector<int64_t> banner_labels = this->get_parameter("banner_labels").as_integer_array();

    this->declare_parameter<std::vector<int64_t>>("banner_numbers", std::vector<int64_t>());
    std::vector<int64_t> banner_numbers = this->get_parameter("banner_numbers").as_integer_array();

    this->declare_parameter<std::vector<bool>>("banner_indicator", std::vector<bool>());
    std::vector<bool> banner_indicator = this->get_parameter("banner_indicator").as_bool_array();

    // Initialize navigation & odometry variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_z = 0;
    m_nav_vx = 0;
    m_nav_vy = 0;
    m_nav_vz = 0;
    m_nav_omega = 0;
    m_nav_heading = 0;
    
    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // update odometry based on transforms on timer click
    odom_timer = this->create_wall_timer(
    std::chrono::duration<float>(((float)1.0)/m_odom_refresh_rate), std::bind(&FactorGraphSLAM::odom_callback, this));

    // publish computed global robot position as a transform slam_map->map and a message with the pose of the robot wrt slam_map
    m_slam_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/tracked", 10);
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // update odometry based on IMU data by subscribing to the odometry message
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", m_odom_queue_size,
    std::bind(&FactorGraphSLAM::odom_msg_callback, this, std::placeholders::_1));

    // Initialize publishers and subscribers
    m_tracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/global", 10);
    m_map_cov_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "obstacle_map/map_cov_viz", 10);
    m_object_sub =
        this->create_subscription<all_seaing_interfaces::msg::ObstacleMap>(
            "detections", m_detection_queue_size,
            std::bind(&FactorGraphSLAM::object_track_map_publish, this, std::placeholders::_1));
    if (m_include_unlabeled){
        m_unlabeled_sub =
            this->create_subscription<all_seaing_interfaces::msg::ObstacleMap>(
                "obstacle_map/unlabeled", m_detection_queue_size,
                std::bind(&FactorGraphSLAM::object_track_map_publish, this, std::placeholders::_1));
    }

    m_restart_service = this->create_service<all_seaing_interfaces::srv::RestartSLAM>("restart_slam", std::bind(&FactorGraphSLAM::restart_slam, this, std::placeholders::_1, std::placeholders::_2));

    if (m_track_banners){
        m_banners_sub = 
            this->create_subscription<all_seaing_interfaces::msg::LabeledObjectPlaneArray>(
                "object_planes", 10,
                std::bind(&FactorGraphSLAM::banners_cb, this, std::placeholders::_1));
        m_tracked_banners_pub = this->create_publisher<all_seaing_interfaces::msg::LabeledObjectPlaneArray>(
            "object_planes/global", 10);
    }

    if (m_match_numbers_indicators){
        for (int i = 0; i < banner_names.size(); i++){
            RCLCPP_INFO(this->get_logger(), "NAME: %s, LABEL: %d, NUMBER: %d, INDICATOR: %s", banner_names[i].c_str(), banner_labels[i], banner_numbers[i], banner_indicator[i]?"TRUE":"FALSE");
            banner_name_to_label[banner_names[i]] = banner_labels[i];
            banner_label_to_name[banner_labels[i]] = banner_names[i];
            banner_label_to_number[banner_labels[i]] = banner_numbers[i];
            banner_label_indicator[banner_labels[i]] = banner_indicator[i];
        }
    }

    m_first_state = true;
    m_got_nav = false;
    m_got_odom = false;
    m_num_obj = 0;
    m_num_banners = 0;

    m_shouldnt_gps_pred = true;
    m_gps_based_predicted = false;
    m_gps_based_dx = 0;
    m_gps_based_dy = 0;
    m_gps_based_dtheta = 0;

    m_obstacle_id = 0;

    // Setting up the noise models for the factor graph
    m_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(m_init_xy_noise, m_init_xy_noise, m_init_theta_noise));
    m_odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(m_imu_xy_noise, m_imu_xy_noise, m_imu_theta_noise));
    m_gps_compass_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(m_update_gps_xy_uncertainty, m_update_gps_xy_uncertainty, m_update_odom_theta_uncertainty));
    m_object_meas_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(m_bearing_std, m_range_std));
    m_banner_meas_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(m_banner_bearing_std, m_banner_range_std, m_banner_phi_std));

    m_isam2 = std::make_shared<gtsam::ISAM2>();
}

void FactorGraphSLAM::restart_slam(const std::shared_ptr<all_seaing_interfaces::srv::RestartSLAM::Request> request,
          std::shared_ptr<all_seaing_interfaces::srv::RestartSLAM::Response> response){

    if (m_first_state) return;

    if (request->restart_position){
        RCLCPP_INFO(this->get_logger(), "RESTARTING SLAM POSITION");
        m_robot_pos_mean = Eigen::Vector3d(m_nav_x, m_nav_y, m_nav_heading);
        m_robot_pos_cov = m_prior_noise->covariance();

        // Reset iSAM2
        // TODO marginalize & measure covariances between last pose & all obstacles & add respective range bearing measurements?
        m_isam2 = std::make_shared<gtsam::ISAM2>();
        m_pose_keys.clear();

        // initialize iSAM2 with the first pose prior
        gtsam::NonlinearFactorGraph graph;
        gtsam::Key first_pose_key = gtsam::Symbol('x', 0);
        m_pose_keys.push_back(first_pose_key);
        graph.add(gtsam::PriorFactor<gtsam::Pose2>(first_pose_key, gtsam::Pose2(m_nav_x, m_nav_y, m_nav_heading), m_prior_noise));
        m_num_poses = 1;
        gtsam::Values initialEstimate;
        initialEstimate.insert(first_pose_key, gtsam::Pose2(m_nav_x, m_nav_y, m_nav_heading));
        m_isam2->update(graph, initialEstimate);

        // TODO change to not remove everything
        m_tracked_obstacles = std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>>>();
        m_num_obj = 0;

        m_tracked_banners = std::vector<std::shared_ptr<all_seaing_perception::Banner>>();
        m_num_banners = 0;

        m_gps_based_predicted = false;
        m_gps_based_dx = 0;
        m_gps_based_dy = 0;
        m_gps_based_dtheta = 0;
        return;
    }

    if (request->restart_buoys){
        RCLCPP_INFO(this->get_logger(), "RESTARTING BUOYS");

        // TODO remove buoy nodes & respective factors from iSAM2
        // need to variable eliminate or only keep last pose & banners similar to how it would be done for resetting positions

        m_tracked_obstacles = std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>>>();
        m_num_obj = 0;
    }

    if (request->restart_banners){
        RCLCPP_INFO(this->get_logger(), "RESTARTING BANNERS");

        // TODO remove banner nodes & respective factors from iSAM2
        // need to variable eliminate or only keep last pose & buoys similar to how it would be done for resetting positions

        m_tracked_banners = std::vector<std::shared_ptr<all_seaing_perception::Banner>>();
        m_num_banners = 0;
    }
}

template <typename T_matrix> std::string matrix_to_string(T_matrix matrix) {
    std::stringstream ss;
    ss << matrix;
    return ss.str();
}

template <typename T_vector> std::string vector_to_string(T_vector v) {
    std::stringstream ss;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i != 0)
            ss << ",";
        ss << v[i];
    }
    return ss.str();
}

void FactorGraphSLAM::publish_slam(){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m_last_nav_time;
    if(m_direct_tf){
        // publish the transform from the local frame (base_link) to slam_map
        // (slam_map is a child of base_link to not interfere with the ekf localization node map)
        t.header.frame_id = m_local_frame_id;
        t.child_frame_id = m_slam_frame_id;
        float inv_x, inv_y, inv_theta;
        std::tie(inv_x, inv_y, inv_theta) = all_seaing_perception::compute_transform_from_to(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2), 0, 0, 0);
        t.transform.translation.x = inv_x;
        t.transform.translation.y = inv_y;
        t.transform.translation.z = -m_nav_z;

        tf2::Quaternion q;
        q.setRPY(0, 0, inv_theta);
        t.transform.rotation = tf2::toMsg(q);
    }else{
        // transform from slam_map to map, s.t. slam_map->robot is predicted pose
        t.header.frame_id = m_slam_frame_id;
        t.child_frame_id = m_global_frame_id;
        double shift_x, shift_y, shift_theta;
        // (slam_map->robot)@(robot->map) = (slam_map->robot)@inv(map->robot)
        std::tie(shift_x, shift_y, shift_theta) =all_seaing_perception::compose_transforms(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2)),all_seaing_perception:: compute_transform_from_to(m_nav_x, m_nav_y, m_nav_heading, 0, 0, 0));
        t.transform.translation.x = shift_x;
        t.transform.translation.y = shift_y;
        t.transform.translation.z = 0;

        tf2::Quaternion q;
        q.setRPY(0, 0, shift_theta);
        t.transform.rotation = tf2::toMsg(q);
    }
    m_tf_broadcaster->sendTransform(t);

    // message
    nav_msgs::msg::Odometry odom_msg = m_last_odom_msg;
    // odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = m_slam_frame_id;
    odom_msg.pose.pose.position.x = m_robot_pos_mean(0);
    odom_msg.pose.pose.position.y = m_robot_pos_mean(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, m_robot_pos_mean(2));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    m_slam_pub->publish(odom_msg);
}

void FactorGraphSLAM::odom_msg_callback(const nav_msgs::msg::Odometry &msg){
    // !!! THOSE ARE RELATIVE TO THE ROBOT AND DEPENDENT ON ITS HEADING, USE THE CORRECT MOTION MODEL
    if(!m_rotate_odom){
        m_nav_vx = msg.twist.twist.linear.x;
        m_nav_vy = msg.twist.twist.linear.y;   
    }else{
        // Pixhawk rotated to the left, facing up, so need to rotate the acceleration vector accordingly
        m_nav_vx = -msg.twist.twist.linear.y;
        m_nav_vy = msg.twist.twist.linear.x;
    }
    m_nav_vz = msg.twist.twist.linear.z;
    m_nav_omega = msg.twist.twist.angular.z;

    m_odom_x = msg.pose.pose.position.x;
    m_odom_y = msg.pose.pose.position.y;
    tf2::Quaternion quat;
    tf2::fromMsg(msg.pose.pose.orientation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p;
    m.getRPY(r, p, m_odom_heading);
    if(r > M_PI/2 || p > M_PI/2){ // to discard weird RPY solutions
        m.getRPY(r, p, m_odom_heading, 2);
    }

    rclcpp::Time curr_odom_time = rclcpp::Time(msg.header.stamp);

    if (!m_got_nav || std::abs((curr_odom_time-m_last_nav_time).seconds()) > m_odom_detection_timeout) return; // make odometry catch up to GPS/current time

    m_last_odom_msg = msg;

    if(!m_got_odom){
        m_last_odom_time = curr_odom_time;
        m_last_odom_x = m_odom_x;
        m_last_odom_y = m_odom_y;
        m_last_odom_heading = m_odom_heading;
        if (std::abs(m_odom_x) < 0.001 || std::abs(m_odom_y) < 0.001 || std::abs(m_odom_heading) < 0.001) return; // probably fake odom/doesn't have correct position (mostly in sim)
        m_got_odom = true;
        return;
    }

    if (!m_got_nav){
        m_last_odom_time = curr_odom_time;
        return;
    }

    // if previous odometry was too old wrt current nav time we've already updated based on gps
    if(m_gps_based_predictions && (std::abs((m_last_odom_time-m_last_nav_time).seconds()) > m_odom_detection_timeout)){
        m_last_odom_time = curr_odom_time;
        return;
    }

    float dt = (curr_odom_time - m_last_odom_time).seconds();
    m_last_odom_time = curr_odom_time;

    if (m_first_state) {
        if (std::abs(m_nav_x) < 0.001 || std::abs(m_nav_y) < 0.001 || std::abs(m_nav_heading) < 0.001) return; // TF not initialized yet
        // initialize mean and cov robot pose
        m_robot_pos_mean = Eigen::Vector3d(m_nav_x, m_nav_y, m_nav_heading);
        m_robot_pos_cov = m_prior_noise->covariance();

        // initialize iSAM2 with the first pose prior
        gtsam::NonlinearFactorGraph graph;
        gtsam::Key first_pose_key = gtsam::Symbol('x', 0);
        m_pose_keys.push_back(first_pose_key);
        graph.add(gtsam::PriorFactor<gtsam::Pose2>(first_pose_key, gtsam::Pose2(m_nav_x, m_nav_y, m_nav_heading), m_prior_noise));
        m_num_poses = 1;
        gtsam::Values initialEstimate;
        initialEstimate.insert(first_pose_key, gtsam::Pose2(m_nav_x, m_nav_y, m_nav_heading));
        m_isam2->update(graph, initialEstimate);

        RCLCPP_INFO(this->get_logger(), "INITIALIZED FACTOR GRAPH");

        m_first_state = false;
        // m_last_odom_time = rclcpp::Time(msg.header.stamp);
        return;
    }

    double dx, dy, dtheta; // in the robot's frame

    if (m_diff_position_odom){
        std::tie(dx, dy, dtheta) = all_seaing_perception::compute_transform_from_to(m_last_odom_x, m_last_odom_y, m_last_odom_heading, m_odom_x, m_odom_y, m_odom_heading);
        if (m_gps_based_predicted){
            if (curr_odom_time > m_last_nav_time){
                // TODO can change it to use compute_transform_from_to right away and nothing else (?)
                std::tie(dx, dy, dtheta) = all_seaing_perception::compose_transforms(all_seaing_perception::compute_transform_from_to(m_gps_based_dx, m_gps_based_dy, m_gps_based_dtheta, 0, 0, 0), std::make_tuple(dx, dy, dtheta));
            }else{
                // odometry is older, no need to use it
                dx = 0;
                dy = 0;
                dtheta = 0;
            }
        }
    }else{
        if (m_gps_based_predicted){
            dt = std::max((curr_odom_time - m_last_nav_time).seconds(), (double)0);
        }
        if (m_nav_omega < 0.001){
            dx = m_nav_vx*dt;
            dy = m_nav_vy*dt;
            dtheta = m_nav_omega*dt;
        }else{
            dx = (sin(m_nav_omega*dt)/m_nav_omega)*m_nav_vx+((cos(m_nav_omega*dt)-1)/m_nav_omega)*m_nav_vy;
            dy = ((1-cos(m_nav_omega*dt))/m_nav_omega)*m_nav_vx+(sin(m_nav_omega*dt)/m_nav_omega)*m_nav_vy;
            dtheta = m_nav_omega*dt;
        }
    }

    if(m_gps_based_predicted){
        m_gps_based_predicted = false;
        m_gps_based_dx = 0;
        m_gps_based_dy = 0;
        m_gps_based_dtheta = 0;
    }

    m_last_odom_x = m_odom_x;
    m_last_odom_y = m_odom_y;
    m_last_odom_heading = m_odom_heading;

    // add new pose to factor graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Key new_pose_key = gtsam::Symbol('x', m_num_poses++);
    m_pose_keys.push_back(new_pose_key);
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(m_pose_keys[m_num_poses-2], new_pose_key, gtsam::Pose2(dx, dy, dtheta), m_odom_noise));
    gtsam::Values initialEstimate;
    gtsam::Pose2 new_estimated_pose = gtsam::Pose2((gtsam::Vector)m_robot_pos_mean)*gtsam::Pose2(dx, dy, dtheta);
    initialEstimate.insert(new_pose_key, new_estimated_pose);
    m_isam2->update(graph, initialEstimate); // this runs the iSAM2 optimization step

    this->update_estimates();

    // if (m_track_robot) {
    //     m_trace.push_back(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), rclcpp::Time(msg.header.stamp).seconds()));
    // }
    
    // // to see the robot position prediction mean & uncertainty
    // if(m_got_nav){
    //     this->visualize_predictions();
    // }
    // // publish robot pose predictions (here and after measurement update) as transforms from map to lidar/camera frame, possibly also as a message
    // // it should ultimately be a source of odometry that can be used by other nodes along with the global map
    // if(m_track_robot && m_got_nav && m_got_odom){
    //     publish_slam();
    // }

    // this->update_maps();

    // this->publish_maps();

    // always call the GPS position measurement update after an odometry prediction, to reduce the effect of noisy odometry to the predicted position & mitigate position jumps due to that
    m_shouldnt_gps_pred = true;
    if (m_gps_update){
        this->odom_callback();
    }
}

void FactorGraphSLAM::gps_based_pred(){
    double dx, dy, dtheta;

    std::tie(dx, dy, dtheta) = all_seaing_perception::compute_transform_from_to(m_last_nav_x, m_last_nav_y, m_last_nav_heading, m_nav_x, m_nav_y, m_nav_heading);

    std::tie(m_gps_based_dx, m_gps_based_dy, m_gps_based_dtheta) = all_seaing_perception::compose_transforms(std::make_tuple(m_gps_based_dx, m_gps_based_dy, m_gps_based_dtheta), std::make_tuple(dx, dy, dtheta));

    // add new pose to factor graph
    gtsam::NonlinearFactorGraph graph;
    gtsam::Key new_pose_key = gtsam::Symbol('x', m_num_poses++);
    m_pose_keys.push_back(new_pose_key);
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(m_pose_keys[m_num_poses-2], new_pose_key, gtsam::Pose2(dx, dy, dtheta), m_odom_noise));
    gtsam::Values initialEstimate;
    gtsam::Pose2 new_estimated_pose = gtsam::Pose2((gtsam::Vector)m_robot_pos_mean)*gtsam::Pose2(dx, dy, dtheta);
    initialEstimate.insert(new_pose_key, new_estimated_pose);
    m_isam2->update(graph, initialEstimate); // this runs the iSAM2 optimization step

    m_gps_based_predicted = true;
}

void FactorGraphSLAM::odom_callback() {
    //update odometry transforms
    //TODO: add a flag for each one that says if they succedeed, to know to continue or not
    m_map_base_link_tf = all_seaing_perception::get_tf(m_tf_buffer, m_global_frame_id, m_local_frame_id);
    m_base_link_map_tf = all_seaing_perception::get_tf(m_tf_buffer, m_local_frame_id, m_global_frame_id);

    m_nav_z = m_map_base_link_tf.transform.translation.z;
    tf2::Quaternion quat;
    tf2::fromMsg(m_map_base_link_tf.transform.rotation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    if(r > M_PI/2 || p > M_PI/2){ // to discard weird RPY solutions
        m.getRPY(r, p, y, 2);
    }

    m_got_nav = true;

    // to ignore gps TFs that are the same thing 5000 times a second and cause SLAM to lock into the GPS
    // if((!m_first_state) && (((m_nav_x == m_map_base_link_tf.transform.translation.x) && (m_nav_y == m_map_base_link_tf.transform.translation.y)) ||  (m_nav_heading == y))) return;

    m_nav_x = m_map_base_link_tf.transform.translation.x;
    m_nav_y = m_map_base_link_tf.transform.translation.y;
    m_nav_heading = y;

    m_last_nav_time = rclcpp::Time(m_map_base_link_tf.header.stamp);

    // RCLCPP_INFO(this->get_logger(), "GOT GPS AT TIME: %lf -> (%lf, %lf, %lf)", m_last_nav_time.seconds(), m_nav_x, m_nav_y, m_nav_heading);

    if(m_first_state) return;

    // if (m_track_robot && (std::abs((rclcpp::Time(m_map_base_link_tf.header.stamp)-m_last_odom_time).seconds()) > m_odom_detection_timeout)) return;

    if (m_shouldnt_gps_pred){
        m_shouldnt_gps_pred = false;
    }else if (m_gps_based_predictions){
        this->gps_based_pred();
    }
    
    if (m_gps_update){
        // add GPS/compass factor to the last pose
        gtsam::NonlinearFactorGraph graph;
        graph.add(all_seaing_perception::UnaryPoseFactor(m_pose_keys.back(), m_nav_x, m_nav_y, m_nav_heading, m_gps_compass_noise, m_include_odom_theta, m_include_odom_only_theta));
        m_isam2->update(graph); // this runs the iSAM2 optimization step
    }

    this->update_estimates();

    m_trace.push_back(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), rclcpp::Time(m_map_base_link_tf.header.stamp).seconds()));

    this->publish_maps();

    if(m_got_nav){
        this->visualize_predictions();
    }

    if(m_got_nav && m_got_odom){
        this->publish_slam();
    }

    this->update_maps();
    this->publish_maps();
    
    m_last_nav_x = m_nav_x;
    m_last_nav_y = m_nav_y;
    m_last_nav_heading = m_nav_heading;
}

template <typename T>
T FactorGraphSLAM::convert_to_global(T point, bool untracked) {
    T new_point = point; // to keep color-related data
    geometry_msgs::msg::Point lc_pt_msg;
    lc_pt_msg.x = point.x;
    lc_pt_msg.y = point.y;
    lc_pt_msg.z = point.z;
    geometry_msgs::msg::Point gb_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(lc_pt_msg, gb_pt_msg, m_map_base_link_tf);
    new_point.x = gb_pt_msg.x;
    new_point.y = gb_pt_msg.y;
    new_point.z = gb_pt_msg.z;
    T act_point = new_point;
    if (!untracked){
        // point initially in map frame
        // want slam_map->map (then will compose it with map->point)
        // (slam_map->robot)@(robot->map) = (slam_map->robot)@inv(map->robot)
        std::tuple<double, double, double> slam_to_map_transform =all_seaing_perception::compose_transforms(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2)),all_seaing_perception::compute_transform_from_to(m_nav_x, m_nav_y, m_nav_heading, 0, 0, 0));
        double th; //uselesss
        std::tie(act_point.x, act_point.y, th) =all_seaing_perception::compose_transforms(slam_to_map_transform, std::make_tuple(new_point.x, new_point.y, 0));
    }
    return act_point;
}

template <typename T>
T FactorGraphSLAM::convert_to_local(T point, bool untracked) {
    T new_point = point; // to keep color-related data
    T act_point = point;
    if (!untracked){
        // point initially in slam_map frame
        // want map->slam_map (then will compose it with slam->point)
        // (map->robot)@(robot->slam_map) = (map->robot)@inv(slam_map->robot)
        std::tuple<double, double, double> map_to_slam_transform =all_seaing_perception::compose_transforms(std::make_tuple(m_nav_x, m_nav_y, m_nav_heading),all_seaing_perception::compute_transform_from_to(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2), 0, 0, 0));
        double th; //uselesss
        std::tie(act_point.x, act_point.y, th) =all_seaing_perception::compose_transforms(map_to_slam_transform, std::make_tuple(new_point.x, new_point.y, 0));
    }
    geometry_msgs::msg::Point gb_pt_msg;
    gb_pt_msg.x = act_point.x;
    gb_pt_msg.y = act_point.y;
    gb_pt_msg.z = act_point.z;
    geometry_msgs::msg::Point lc_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(gb_pt_msg, lc_pt_msg, m_base_link_map_tf);
    new_point.x = lc_pt_msg.x;
    new_point.y = lc_pt_msg.y;
    new_point.z = lc_pt_msg.z;
    return new_point;
}

void FactorGraphSLAM::update_estimates(){
    gtsam::Pose2 robot_pose = m_isam2->calculateEstimate(m_pose_keys.back()).cast<gtsam::Pose2>();
    m_robot_pos_mean = Eigen::Vector3d(robot_pose.x(), robot_pose.y(), robot_pose.theta());
    // m_robot_pos_mean = (Eigen::Vector3d)gtsam::Pose2::Logmap(m_isam2->calculateEstimate(m_pose_keys.back()).cast<gtsam::Pose2>());
    m_robot_pos_cov = (Eigen::Matrix3d)m_isam2->marginalCovariance(m_pose_keys.back());
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        m_tracked_obstacles[i]->mean_pred = (Eigen::Vector2d)((gtsam::Point2)m_isam2->calculateEstimate(m_tracked_obstacles[i]->node_key).cast<gtsam::Point2>());
        m_tracked_obstacles[i]->cov = (Eigen::Matrix2d)m_isam2->marginalCovariance(m_tracked_obstacles[i]->node_key);
    }
    if (m_track_banners){
        for (size_t i = 0; i < m_tracked_banners.size(); i++) {
            gtsam::Pose2 banner_pose = m_isam2->calculateEstimate(m_tracked_banners[i]->node_key).cast<gtsam::Pose2>();
            m_tracked_banners[i]->mean_pred = Eigen::Vector3d(banner_pose.x(), banner_pose.y(), banner_pose.theta());
            // m_tracked_banners[i]->mean_pred = (Eigen::Vector3d)gtsam::Pose2::Logmap(m_isam2->calculateEstimate(m_tracked_banners[i]->node_key).cast<gtsam::Pose2>());
            m_tracked_banners[i]->cov = (Eigen::Matrix3d)m_isam2->marginalCovariance(m_tracked_banners[i]->node_key);
        }
    }
    // TODO update the trace with pose estimates from the factor graph (last same # of poses in the trace at the time) every once in a while
    // for visualization purposes
}

void FactorGraphSLAM::update_maps(){
    // Update all (new and old, since they also have correlation with each other) objects global
    // positions (including the cloud, and then the local points respectively)
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        pcl::PointXYZHSV prev_glob_centr = m_tracked_obstacles[i]->obstacle.get_global_point();
        pcl::PointXYZHSV upd_glob_centr = prev_glob_centr;
        upd_glob_centr.x = m_tracked_obstacles[i]->mean_pred[0];
        upd_glob_centr.y = m_tracked_obstacles[i]->mean_pred[1];
        m_tracked_obstacles[i]->obstacle.global_transform(upd_glob_centr.x-prev_glob_centr.x, upd_glob_centr.y-prev_glob_centr.y, 0);
        double dx, dy, dtheta;
        std::tie(dx, dy, dtheta) = all_seaing_perception::compose_transforms(std::make_tuple(m_nav_x, m_nav_y, m_nav_heading),all_seaing_perception::compute_transform_from_to(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2), 0, 0, 0));
        geometry_msgs::msg::Transform map_to_slam_tf;
        tf2::Transform map_to_slam_tf_tf, base_link_map_tf_tf;
        map_to_slam_tf.translation.x = dx;
        map_to_slam_tf.translation.y = dy;
        tf2::Quaternion q;
        q.setRPY(0, 0, dtheta);
        map_to_slam_tf.rotation = tf2::toMsg(q);
        tf2::fromMsg(map_to_slam_tf, map_to_slam_tf_tf);
        tf2::fromMsg(m_base_link_map_tf.transform, base_link_map_tf_tf);
        geometry_msgs::msg::TransformStamped comb_trans;
        comb_trans.transform = tf2::toMsg(base_link_map_tf_tf*map_to_slam_tf_tf);
        comb_trans.header = m_local_header;
        comb_trans.child_frame_id = m_global_header.frame_id;
        m_tracked_obstacles[i]->obstacle.global_to_local(m_local_header, comb_trans);
    }

    if (!m_track_banners) return;
    for (size_t i = 0; i < m_tracked_banners.size(); i++) {
        m_tracked_banners[i]->plane_msg.normal_ctr.position.x = m_tracked_banners[i]->mean_pred[0];
        m_tracked_banners[i]->plane_msg.normal_ctr.position.y = m_tracked_banners[i]->mean_pred[1];
        tf2::Quaternion q;
        q.setRPY(0, 0, m_tracked_banners[i]->mean_pred[2]);
        m_tracked_banners[i]->plane_msg.normal_ctr.orientation = tf2::toMsg(q);
    }
}

void FactorGraphSLAM::publish_maps(){
    // Publish map with tracked obstacles
    all_seaing_interfaces::msg::ObstacleMap map_msg;
    map_msg.ns = "global";
    map_msg.local_header = m_local_header;
    map_msg.header = m_global_header;
    map_msg.is_labeled = true;
    for (std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>> t_ob : m_tracked_obstacles) {
        all_seaing_interfaces::msg::Obstacle det_obstacle;
        t_ob->obstacle.to_ros_msg(det_obstacle);
        det_obstacle.label = t_ob->label;
        map_msg.obstacles.push_back(det_obstacle);
    }
    m_tracked_map_pub->publish(map_msg);

    if (m_track_banners){
        // Publish tracked banners
        all_seaing_interfaces::msg::LabeledObjectPlaneArray plane_arr_msg;
        plane_arr_msg.header = m_global_header;
        for (std::shared_ptr<all_seaing_perception::Banner> t_ob : m_tracked_banners) {
            plane_arr_msg.objects.push_back(t_ob->plane_msg);
        }
        m_tracked_banners_pub->publish(plane_arr_msg);
    }
}

void FactorGraphSLAM::visualize_predictions() {
    // delete previous markers
    visualization_msgs::msg::MarkerArray delete_arr;
    visualization_msgs::msg::Marker delete_mark;
    delete_mark.id = 0;
    delete_mark.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_arr.markers.push_back(delete_mark);
    m_map_cov_viz_pub->publish(delete_arr);

    visualization_msgs::msg::MarkerArray ellipse_arr;
    // robot pose prediction
    Eigen::Vector2d robot_mean = m_robot_pos_mean.segment(0, 2);
    Eigen::Matrix2d robot_cov = m_robot_pos_cov.block(0, 0, 2, 2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(robot_cov);
    double a_x = eigen_solver.eigenvalues()(0);
    double a_y = eigen_solver.eigenvalues()(1);
    Eigen::Vector2d axis_x = eigen_solver.eigenvectors().col(0);
    Eigen::Vector2d axis_y = eigen_solver.eigenvectors().col(1);
    tf2::Quaternion quat_rot;
    quat_rot.setRPY(0,0,std::atan2(axis_x(1), axis_x(0)));
    visualization_msgs::msg::Marker ellipse;
    ellipse.type = visualization_msgs::msg::Marker::SPHERE;
    ellipse.pose.position.x = robot_mean(0);
    ellipse.pose.position.y = robot_mean(1);
    ellipse.pose.position.z = 0;
    ellipse.pose.orientation = tf2::toMsg(quat_rot);
    ellipse.scale.x = sqrt(a_x);
    ellipse.scale.y = sqrt(a_y);
    ellipse.scale.z = 0.5;
    ellipse.color.a = 0.5;
    ellipse.color.g = 1;
    ellipse.header = m_global_header;
    ellipse.id = m_num_obj + 3*m_num_banners + 1;
    ellipse_arr.markers.push_back(ellipse);

    visualization_msgs::msg::Marker angle_marker;
    angle_marker.type = visualization_msgs::msg::Marker::ARROW;
    angle_marker.pose.position.x = robot_mean(0);
    angle_marker.pose.position.y = robot_mean(1);
    angle_marker.pose.position.z = 0;
    tf2::Quaternion angle_quat;
    angle_quat.setRPY(0, 0, m_robot_pos_mean(2));
    angle_marker.pose.orientation = tf2::toMsg(angle_quat);
    angle_marker.scale.x = sqrt(m_robot_pos_cov(2, 2));
    angle_marker.scale.y = 0.2;
    angle_marker.scale.z = 0.2;
    angle_marker.color.a = 1;
    angle_marker.header = m_global_header;
    angle_marker.id = m_num_obj + 3*m_num_banners + 2;
    ellipse_arr.markers.push_back(angle_marker);

    // obstacle predictions
    for (int i = 0; i < m_num_obj; i++) {
        Eigen::Vector2d obj_mean = m_tracked_obstacles[i]->mean_pred;
        Eigen::Matrix2d obj_cov = m_tracked_obstacles[i]->cov;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(obj_cov);
        double a_x = eigen_solver.eigenvalues()(0);
        double a_y = eigen_solver.eigenvalues()(1);
        Eigen::Vector2d axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2d axis_y = eigen_solver.eigenvectors().col(1);
        tf2::Quaternion quat_rot;
        quat_rot.setRPY(0,0,std::atan2(axis_x(1), axis_x(0)));
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;

        ellipse.pose.position.x = obj_mean(0);
        ellipse.pose.position.y = obj_mean(1);
        ellipse.pose.orientation = tf2::toMsg(quat_rot);
        
        ellipse.pose.position.z = 0;
        ellipse.scale.x = sqrt(a_x);
        ellipse.scale.y = sqrt(a_y);
        ellipse.scale.z = 1;
        ellipse.color.a = 0.2;
        ellipse.color.r = 1;
        ellipse.header = m_global_header;
        ellipse.id = i;
        ellipse_arr.markers.push_back(ellipse);
    }

    if (m_track_banners){
        // banner visualization
        for (int i = 0; i < m_num_banners; i++){
            Eigen::Vector2d obj_mean;
            Eigen::Matrix2d obj_cov;
            float theta, theta_cov;
            obj_mean = m_tracked_banners[i]->mean_pred.segment(0, 2);
            obj_cov = m_tracked_banners[i]->cov.block(0, 0, 2, 2);
            theta = m_tracked_banners[i]->mean_pred(2);
            theta_cov = m_tracked_banners[i]->cov(2,2);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(obj_cov);
            double a_x = eigen_solver.eigenvalues()(0);
            double a_y = eigen_solver.eigenvalues()(1);
            Eigen::Vector2d axis_x = eigen_solver.eigenvectors().col(0);
            Eigen::Vector2d axis_y = eigen_solver.eigenvectors().col(1);
            tf2::Quaternion quat_rot;
            quat_rot.setRPY(0,0,std::atan2(axis_x(1), axis_x(0)));
            visualization_msgs::msg::Marker banner_ellipse;
            banner_ellipse.type = visualization_msgs::msg::Marker::SPHERE;
            banner_ellipse.pose.position.x = obj_mean(0);
            banner_ellipse.pose.position.y = obj_mean(1);
            banner_ellipse.pose.position.z = 0;
            banner_ellipse.pose.orientation = tf2::toMsg(quat_rot);
            banner_ellipse.scale.x = sqrt(a_x);
            banner_ellipse.scale.y = sqrt(a_y);
            banner_ellipse.scale.z = 0.5;
            banner_ellipse.color.a = 0.5;
            banner_ellipse.color.b = 1;
            banner_ellipse.header = m_global_header;
            banner_ellipse.id = m_num_obj + 3*i;
            ellipse_arr.markers.push_back(banner_ellipse);

            visualization_msgs::msg::Marker banner_angle_marker;
            banner_angle_marker.type = visualization_msgs::msg::Marker::ARROW;
            banner_angle_marker.pose.position.x = obj_mean(0);
            banner_angle_marker.pose.position.y = obj_mean(1);
            banner_angle_marker.pose.position.z = 0;
            tf2::Quaternion angle_quat;
            angle_quat.setRPY(0, 0, theta);
            banner_angle_marker.pose.orientation = tf2::toMsg(angle_quat);
            banner_angle_marker.scale.x = sqrt(theta_cov);
            banner_angle_marker.scale.y = 0.2;
            banner_angle_marker.scale.z = 0.2;
            banner_angle_marker.color.a = 1;
            banner_angle_marker.header = m_global_header;
            banner_angle_marker.id = m_num_obj + 3*i + 1;
            ellipse_arr.markers.push_back(banner_angle_marker);

            visualization_msgs::msg::Marker banner_label_marker;
            banner_label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            banner_label_marker.pose.position.x = obj_mean(0);
            banner_label_marker.pose.position.y = obj_mean(1);
            banner_label_marker.pose.position.z = 0;
            banner_label_marker.scale.z = 2.0;
            banner_label_marker.color.a = 1;
            banner_label_marker.color.r = 1;
            banner_label_marker.color.g = 1;
            banner_label_marker.text = std::to_string(m_tracked_banners[i]->label);
            banner_label_marker.header = m_global_header;
            banner_label_marker.id = m_num_obj + 3*i + 2;
            ellipse_arr.markers.push_back(banner_label_marker);
        }
    }

    visualization_msgs::msg::Marker trace;
    trace.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trace.header = m_global_header;
    trace.id = m_num_obj + 3*m_num_banners + 3;
    trace.scale.x = 0.1;
    trace.scale.y = 0.1;
    trace.color.a = 1;
    trace.color.r = 1;
    trace.color.g = 1;
    // clean up old points from trace
    while(!m_trace.empty() && (std::get<2>(m_trace.back()) - std::get<2>(m_trace.front())) > m_trace_time){
        m_trace.pop_front();
    }
    for (std::tuple<float, float, float> p : m_trace){
        geometry_msgs::msg::Point pt = geometry_msgs::msg::Point();
        float time;
        std::tie(pt.x, pt.y, time) = p;
        trace.points.push_back(pt);
    }
    ellipse_arr.markers.push_back(trace);

    m_map_cov_viz_pub->publish(ellipse_arr);
}

void FactorGraphSLAM::object_track_map_publish(const all_seaing_interfaces::msg::ObstacleMap::ConstSharedPtr &msg){
    // Set up headers and transforms
    m_local_header = msg->local_header;
    m_global_header.frame_id = m_slam_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    std_msgs::msg::Header m_global_untracked_header = m_global_header;
    m_global_untracked_header.frame_id = m_global_frame_id;
    m_local_frame_id = m_local_header.frame_id;

    // m_map_base_link_tf = all_seaing_perception::get_tf(m_tf_buffer, m_global_frame_id, m_local_frame_id);
    // m_base_link_map_tf = all_seaing_perception::get_tf(m_tf_buffer, m_local_frame_id, m_global_frame_id);

    if(m_first_state) return;
    
    // RCLCPP_INFO(this->get_logger(), "DIFFERENCE BETWEEN DETECTION & LAST ODOM TIME: %lf - %lf = %lf", rclcpp::Time(msg->local_header.stamp).nanoseconds()/((float)1e9), m_last_odom_time.nanoseconds()/((float)1e9), std::abs((rclcpp::Time(msg->local_header.stamp)-m_last_odom_time).seconds()));

    // if (m_track_robot && (std::abs((rclcpp::Time(msg->local_header.stamp)-m_last_odom_time).seconds()) > m_odom_detection_timeout)) return;

    if (m_gps_update && std::abs((rclcpp::Time(msg->local_header.stamp)-m_last_nav_time).seconds()) > m_odom_detection_timeout) return; // make detections catch up to GPS/current time

    this->odom_callback(); //catch up to current position before applying sensor data

    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // // measuring time
    // using std::chrono::high_resolution_clock;
    // using std::chrono::duration_cast;
    // using std::chrono::duration;
    // using std::chrono::milliseconds;

    // auto t1 = high_resolution_clock::now();
    // auto t2 = high_resolution_clock::now();
    // duration<double, std::milli> ms_double = t2 - t1;

    // t1 = high_resolution_clock::now();

    std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>>> detected_obstacles;
    for (all_seaing_interfaces::msg::Obstacle obs : msg->obstacles) {
        std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>> obj_cloud(new all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>(rclcpp::Time(m_local_header.stamp), obs.label, all_seaing_perception::Obstacle<pcl::PointXYZHSV>(m_local_header, m_global_header, obs)));
        obj_cloud->obstacle.local_to_global(m_global_header, m_map_base_link_tf);
        detected_obstacles.push_back(obj_cloud);
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "STORING DETECTIONS: %lfms", ms_double.count());

    // t1 = high_resolution_clock::now();

    Eigen::Matrix<double, 2, 2> Q{
        {m_range_std*m_range_std, 0},
        {0, m_bearing_std*m_bearing_std},
    };
    std::vector<std::vector<float>> p;
    std::vector<float> v_indiv;
    std::vector<std::vector<float>> v_meas;
    for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
        v_indiv.push_back(m_tracked_obstacles[tracked_id]->cov.trace());
    }
    
    // TODO make this a batch operation (stack covariance matrices & errors w/ each obstacle & compute all at once, make sure Eigen is using GPU if possible)

    for (std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>> det_obs : detected_obstacles) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) =
            all_seaing_perception::local_to_range_bearing_signature(det_obs->obstacle.get_local_point(), det_obs->label);
        p.push_back(std::vector<float>());
        v_meas.push_back(std::vector<float>());
        Eigen::Vector2d z_pred;
        Eigen::MatrixXd Psi;
        for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
            // if (m_track_robot) {
            //     float d_x = m_state(3 + 2 * tracked_id) - m_state(0);
            //     float d_y = m_state(3 + 2 * tracked_id + 1) - m_state(1);
            //     float q = d_x * d_x + d_y * d_y;
            //     z_pred = Eigen::Vector2f(std::sqrt(q), all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - m_state(2)));
            //     Eigen::MatrixXf F = Eigen::MatrixXf::Zero(5, m_mat_size);
            //     F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
            //     F.block(3, 3 + 2 * tracked_id, 2, 2) = Eigen::Matrix2f::Identity();
            //     Eigen::Matrix<float, 2, 5> h{
            //         {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x,
            //          std::sqrt(q) * d_y},
            //         {d_y, -d_x, -q, -d_y, d_x},
            //     };
            //     // Do not store vectors, since they don't compute covariance with other obstacles in
            //     // the same detection batch
            //     Eigen::MatrixXf H = h * F / q;

            //     Psi = H * m_cov * H.transpose() + Q;
            // } else {

            // TODO marginalize everything except last robot pos & the landmark we are considering to get covariances of those (INCLUDING BETWEEN THEM)
            // & use the above formula to compute a more accurate Mahalanobis distance
            // TODO try to get the updated (based on the factor graph, not only the factor) noise model between the pose & the landmark from GTSAM
            // and also the error used in the BearingRange factor & use those to compute the Mahalanobis distance
            float d_x = m_tracked_obstacles[tracked_id]->mean_pred[0] - m_robot_pos_mean(0);
            float d_y = m_tracked_obstacles[tracked_id]->mean_pred[1] - m_robot_pos_mean(1);
            float q = d_x * d_x + d_y * d_y;
            z_pred = Eigen::Vector2d(std::sqrt(q), all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - m_robot_pos_mean(2)));

            Eigen::Matrix<double, 2, 2> h{
                {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                {-d_y, d_x},
            };
            // Do not store vectors, since they don't compute covariance with other obstacles in
            // the same detection batch
            Eigen::MatrixXd H = h / q;
            Psi = H * m_tracked_obstacles[tracked_id]->cov * H.transpose() + Q;

            Eigen::Vector2d z_actual(range, bearing);

            z_actual(1) = z_pred(1)+all_seaing_perception::angle_to_pi_range(z_actual(1)-z_pred(1));
            p.back().push_back((z_actual - z_pred).transpose() * Psi.inverse() *
                               (z_actual - z_pred));
            v_meas.back().push_back(Psi.trace());
        }
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "UNKNOWN ASSOCIATIONS: %lfms", ms_double.count()); // TODO OPTIMIZE, IT'S TAKING MUCH LONGER THAN THE OTHER STUFF

    // t1 = high_resolution_clock::now();

    std::vector<int> match;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    double assoc_threshold = msg->is_labeled?m_new_obj_slam_thres:m_unlabeled_assoc_threshold;
    if (m_data_association_algo == "greedy_exclusive"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_data_association(m_tracked_obstacles, detected_obstacles, p, assoc_threshold);
    }else if (m_data_association_algo == "greedy_individual"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::indiv_greedy_data_association(m_tracked_obstacles, detected_obstacles, p, assoc_threshold);
    }else if (m_data_association_algo == "greedy_exclusive_indiv_var"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_indiv_var_data_association(m_tracked_obstacles, detected_obstacles, p, v_indiv, assoc_threshold);
    }else if (m_data_association_algo == "greedy_exclusive_measurement_var"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_meas_var_data_association(m_tracked_obstacles, detected_obstacles, p, v_meas, assoc_threshold);
    }else if (m_data_association_algo == "linear_sum_assignment"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::linear_sum_assignment_data_association(m_tracked_obstacles, detected_obstacles, p, assoc_threshold);
    }else if (m_data_association_algo == "linear_sum_assignment_sqrt"){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::linear_sum_assignment_data_association(m_tracked_obstacles, detected_obstacles, p, assoc_threshold, true);
    }else{
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_data_association(m_tracked_obstacles, detected_obstacles, p, assoc_threshold);
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "ASSOCIATIONS: %lfms", ms_double.count());

    // t1 = high_resolution_clock::now();

    // add new factors to the factor graph w/ known correspondences
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate; // only for new objects
    for (size_t i = 0; i < detected_obstacles.size(); i++) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = all_seaing_perception::local_to_range_bearing_signature(
            detected_obstacles[i]->obstacle.get_local_point(), detected_obstacles[i]->label);
        Eigen::Vector2d z_actual(range, bearing);

        if (match[i] == -1) {
            if (detected_obstacles[i]->label == -1){
                // don't create new obstacles when they have no label
                continue;
            }
            // increase object count and expand & initialize matrices
            m_num_obj++;
            // add object to tracked obstacles vector
            detected_obstacles[i]->obstacle.set_id(m_obstacle_id++);
            m_tracked_obstacles.push_back(detected_obstacles[i]);

            gtsam::Key new_object_key;
            new_object_key = gtsam::Symbol('l', m_obstacle_id);
            m_tracked_obstacles.back()->node_key = new_object_key;
            gtsam::Point2 new_obj_estimated_pos = Eigen::Vector2d(m_robot_pos_mean(0), m_robot_pos_mean(1)) +
                range * Eigen::Vector2d(std::cos(bearing + m_robot_pos_mean(2)),
                                        std::sin(bearing + m_robot_pos_mean(2)));
            initialEstimate.insert(new_object_key, new_obj_estimated_pos);
        }
        int tracked_id = match[i] >= 0 ? match[i] : m_num_obj - 1;

        // add the range bearing factor
        graph.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(m_pose_keys.back(), m_tracked_obstacles[tracked_id]->node_key, gtsam::Rot2(bearing), range, m_object_meas_noise));

        // update data for matched obstacles (we'll update position after we update SLAM with all points)
        detected_obstacles[i]->obstacle.set_id(m_tracked_obstacles[tracked_id]->obstacle.get_id());
        detected_obstacles[i]->label = m_tracked_obstacles[tracked_id]->label;
        detected_obstacles[i]->time_seen = m_tracked_obstacles[tracked_id]->time_seen;
        detected_obstacles[i]->last_dead = m_tracked_obstacles[tracked_id]->last_dead;
        detected_obstacles[i]->time_dead = m_tracked_obstacles[tracked_id]->time_dead;
        detected_obstacles[i]->is_dead = m_tracked_obstacles[tracked_id]->is_dead;
        detected_obstacles[i]->node_key = m_tracked_obstacles[tracked_id]->node_key;
        m_tracked_obstacles[tracked_id] = detected_obstacles[i];
    }

    m_isam2->update(graph, initialEstimate); // this runs the iSAM2 optimization step

    // update the means & covariances of everything
    this->update_estimates();

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "UPDATE WITH KNOWN ASSOCIATIONS: %lfms", ms_double.count()); // TODO OPTIMIZE, IT'S TAKING MUCH LONGER THAN THE OTHER STUFF

    // t1 = high_resolution_clock::now();
    
    this->update_maps();

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "UPDATE MAPS: %lfms", ms_double.count());

    pcl::PointXYZ p0(0, 0, 0);
    float avg_dist = 0;
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        avg_dist += pcl::euclideanDistance(p0, m_tracked_obstacles[i]->obstacle.get_local_point()) / ((float)m_tracked_obstacles.size());
    }

    // t1 = high_resolution_clock::now();

    // Filter old obstacles
    std::unordered_set<int> to_keep_set;
    for (int tracked_id = 0; tracked_id < static_cast<int>(m_tracked_obstacles.size()); tracked_id++) {
        if (chosen_tracked.count(tracked_id) && ((!m_drop_ignore_unlabeled) || msg->is_labeled)) {
            to_keep_set.insert(tracked_id);
            m_tracked_obstacles[tracked_id]->is_dead = false;
            m_tracked_obstacles[tracked_id]->time_dead = rclcpp::Duration(0,0);
            if(msg->is_labeled){
                m_tracked_obstacles[tracked_id]->time_seen = m_local_header.stamp;
            }
            continue;
        }
        // Dead
        if (m_tracked_obstacles[tracked_id]->is_dead) {
            // Was also dead before, add time dead
            m_tracked_obstacles[tracked_id]->time_dead =
                rclcpp::Time(m_local_header.stamp) -
                m_tracked_obstacles[tracked_id]->last_dead +
                m_tracked_obstacles[tracked_id]->time_dead;
            if ((m_tracked_obstacles[tracked_id]->time_dead.seconds() >
                (m_normalize_drop_thresh ? (m_obstacle_drop_thresh * (pcl::euclideanDistance(p0,
                                            m_tracked_obstacles[tracked_id]->obstacle.get_local_point()) / avg_dist) *
                                            m_normalize_drop_dist) : m_obstacle_drop_thresh)) && (pcl::euclideanDistance(p0,
                                            m_tracked_obstacles[tracked_id]->obstacle.get_local_point()) < m_range_drop_thresh)) {
                continue;
            }
        }
        m_tracked_obstacles[tracked_id]->is_dead = true;
        m_tracked_obstacles[tracked_id]->last_dead = m_local_header.stamp;
        to_keep_set.insert(tracked_id);
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "FILTERING OLD OBSTACLES: %lfms", ms_double.count());

    // t1 = high_resolution_clock::now();

    // Clear out duplicates
    // TODO: optimize to N^2 (now is N^3) by making a priority queue of pairs of indices based on minimum distance and popping if index already removed and removing if distance < threshold
    int ind_to_remove = 0;
    while(ind_to_remove != -1){
        ind_to_remove = -1;
        // find minimum distance obstacles and remove one seen earliest if under the duplicate threshold
        float min_dist = m_duplicate_thresh;
        for (int i : to_keep_set){
            for (int j : to_keep_set){
                if (i == j){
                    continue;
                }
                if (pcl::euclideanDistance(m_tracked_obstacles[i]->obstacle.get_global_point(),
                                            m_tracked_obstacles[j]->obstacle.get_global_point()) < min_dist){
                    min_dist = pcl::euclideanDistance(m_tracked_obstacles[i]->obstacle.get_global_point(),
                                                        m_tracked_obstacles[j]->obstacle.get_global_point());
                    // remove
                    if (m_tracked_obstacles[i]->label == m_tracked_obstacles[j]->label){
                        ind_to_remove = m_tracked_obstacles[i]->cov.trace() < m_tracked_obstacles[j]->cov.trace()?j:i;
                    }else{
                        ind_to_remove = (m_tracked_obstacles[i]->time_seen < m_tracked_obstacles[j]->time_seen)?i:j;
                    }
                }
            }
        }
        if (ind_to_remove != -1){
            to_keep_set.erase(ind_to_remove);
        }
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "CLEARING OUT DUPLICATES: %lfms", ms_double.count());

    // t1 = high_resolution_clock::now();

    std::vector<int> to_remove;
    std::vector<int> to_keep;
    for (int i = 0; i < m_tracked_obstacles.size(); i++){
        if(to_keep_set.count(i)){
            to_keep.push_back(i);
        }else{
            to_remove.push_back(i);
        }
    }

    // update vectors & matrices
    if (!to_remove.empty()) {
        std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud<pcl::PointXYZHSV>>> new_obj;

        for (int i : to_keep) {
            new_obj.push_back(m_tracked_obstacles[i]);
        }
        
        // can't remove from GTSAM factor graph because it affects the belief
        // TODO apply variable elimination (marginalize then remove from the graph), see how to do it in GTSAM

        m_tracked_obstacles = new_obj;
        m_num_obj = to_keep.size();
    }

    // t2 = high_resolution_clock::now();

    // ms_double = t2 - t1;
    
    // RCLCPP_INFO(this->get_logger(), "REMOVING FILTERED OBSTACLES FROM MATRICES: %lfms", ms_double.count());

    m_trace.push_back(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), rclcpp::Time(m_local_header.stamp).seconds()));

    this->publish_maps();

    if(m_got_nav){
        this->visualize_predictions();
    }
    if(m_got_nav && m_got_odom){
        this->publish_slam();
    }
}

void FactorGraphSLAM::banners_cb(const all_seaing_interfaces::msg::LabeledObjectPlaneArray::ConstSharedPtr &msg){
    // Set up headers and transforms
    m_local_header = msg->header;
    m_global_header.frame_id = m_slam_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    std_msgs::msg::Header m_global_untracked_header = m_global_header;
    m_global_untracked_header.frame_id = m_global_frame_id;
    m_local_frame_id = m_local_header.frame_id;

    m_map_base_link_tf = all_seaing_perception::get_tf(m_tf_buffer, m_global_frame_id, m_local_frame_id);
    m_base_link_map_tf = all_seaing_perception::get_tf(m_tf_buffer, m_local_frame_id, m_global_frame_id);

    if(m_first_state) return;

    if (m_gps_update && std::abs((rclcpp::Time(msg->header.stamp)-m_last_nav_time).seconds()) > m_odom_detection_timeout) return; // make detections catch up to GPS/current time

    this->odom_callback(); //catch up to current position before applying sensor data

    std::vector<std::shared_ptr<all_seaing_perception::Banner>> detected_banners;
    for (all_seaing_interfaces::msg::LabeledObjectPlane banner_msg : msg->coplanar_indiv) {
        std::shared_ptr<all_seaing_perception::Banner> banner(new all_seaing_perception::Banner(rclcpp::Time(m_local_header.stamp), banner_msg.label, banner_msg));
        detected_banners.push_back(banner);
    }

    Eigen::Matrix<double, 3, 3> Q{
        {m_banner_range_std*m_banner_range_std, 0, 0},
        {0, m_banner_bearing_std*m_banner_bearing_std, 0},
        {0, 0, m_banner_phi_std*m_banner_phi_std},
    };
    std::vector<std::vector<float>> p;
    std::vector<float> v_indiv;
    std::vector<std::vector<float>> v_meas;
    for (int tracked_id = 0; tracked_id < m_num_banners; tracked_id++) {
        v_indiv.push_back(m_tracked_banners[tracked_id]->cov.trace());
    }

    // TODO make this a batch operation (stack covariance matrices & errors w/ each obstacle & compute all at once, make sure Eigen is using GPU if possible)
    for (std::shared_ptr<all_seaing_perception::Banner> det_obs : detected_banners) {
        float range, bearing, phi;
        int signature;
        std::tie(range, bearing, phi, signature) =
            all_seaing_perception::local_banner_to_range_bearing_signature(det_obs->plane_msg.normal_ctr, det_obs->label);
        p.push_back(std::vector<float>());
        v_meas.push_back(std::vector<float>());
        Eigen::Vector3d z_pred;
        Eigen::MatrixXd Psi;
        for (int tracked_id = 0; tracked_id < m_num_banners; tracked_id++) {
            // if (m_track_robot && m_banners_slam) {
            //     float d_x = m_state(3 + 2*m_num_obj + 3 * tracked_id) - m_state(0);
            //     float d_y = m_state(3 + 2*m_num_obj + 3 * tracked_id + 1) - m_state(1);
            //     float d_theta = m_state(3 + 2*m_num_obj + 3 * tracked_id + 2) - m_state(2);
            //     float q = d_x * d_x + d_y * d_y;
            //     z_pred = Eigen::Vector3f(std::sqrt(q), all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - m_state(2)), all_seaing_perception::mod_2pi(d_theta));
            //     Eigen::MatrixXf F = Eigen::MatrixXf::Zero(6, m_mat_size);
            //     F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
            //     F.block(3, 3 + 2*m_num_obj + 3 * tracked_id, 3, 3) = Eigen::Matrix3f::Identity();
            //     Eigen::Matrix<float, 3, 6> h{
            //         {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x,
            //          std::sqrt(q) * d_y, 0},
            //         {d_y, -d_x, -q, -d_y, d_x, 0},
            //         {0, 0, -q, 0, 0, q},
            //     };
            //     // Do not store vectors, since they don't compute covariance with other obstacles in
            //     // the same detection batch
            //     Eigen::MatrixXf H = h * F / q;

            //     Psi = H * m_cov * H.transpose() + Q;
            // } else {

            // TODO marginalize everything except last robot pos & the landmark we are considering to get covariances of those (INCLUDING BETWEEN THEM)
            // & use the above formula to compute a more accurate Mahalanobis distance
            // TODO try to get the updated (based on the factor graph, not only the factor) noise model between the pose & the landmark from GTSAM
            // and also the error used in the BearingRange factor & use those to compute the Mahalanobis distance
            float d_x = m_tracked_banners[tracked_id]->mean_pred[0] - m_robot_pos_mean(0);
            float d_y = m_tracked_banners[tracked_id]->mean_pred[1] - m_robot_pos_mean(1);
            float d_theta = m_tracked_banners[tracked_id]->mean_pred[2] - m_robot_pos_mean(2);
            float q = d_x * d_x + d_y * d_y;
            z_pred = Eigen::Vector3d(std::sqrt(q), all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - m_robot_pos_mean(2)), all_seaing_perception::mod_2pi(d_theta));
            Eigen::Matrix<double, 3, 3> h{
                {std::sqrt(q) * d_x, std::sqrt(q) * d_y, 0},
                {-d_y, d_x, 0},
                {0, 0, q},
            };
            // Do not store vectors, since they don't compute covariance with other obstacles in
            // the same detection batch
            Eigen::MatrixXd H = h / q;
            Psi = H * m_tracked_banners[tracked_id]->cov * H.transpose() + Q;

            Eigen::Vector3d z_actual(range, bearing, phi);

            z_actual(1) = z_pred(1)+all_seaing_perception::angle_to_pi_range(z_actual(1)-z_pred(1));
            z_actual(2) = z_pred(2)+all_seaing_perception::angle_to_pi_range(z_actual(2)-z_pred(2));
            p.back().push_back((z_actual - z_pred).transpose() * Psi.inverse() *
                               (z_actual - z_pred));
            v_meas.back().push_back(Psi.trace());
        }
    }

    std::vector<int> match;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    double assoc_threshold = m_new_banner_slam_thres;
    if (m_match_numbers_indicators){
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_banner_data_association(m_tracked_banners, detected_banners, p, assoc_threshold, banner_label_to_number);
    }else{
        std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_banner_data_association(m_tracked_banners, detected_banners, p, assoc_threshold);
    }
    
    // add new factors to the factor graph w/ known correspondences
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate; // only for new objects
    for (size_t i = 0; i < detected_banners.size(); i++) {
        float range, bearing, phi;
        int signature;
        std::tie(range, bearing, phi, signature) =
            all_seaing_perception::local_banner_to_range_bearing_signature(detected_banners[i]->plane_msg.normal_ctr, detected_banners[i]->label);
        Eigen::Vector3d z_actual(range, bearing, phi);
        // if (match[i] != -1){
        //     float d_x = m_state(3 + 2*m_num_obj + 3 * match[i]) - m_state(0);
        //     float d_y = m_state(3 + 2*m_num_obj + 3 * match[i] + 1) - m_state(1);
        //     float d_theta = m_state(3 + 2*m_num_obj + 3 * match[i] + 2) - m_state(2);
        //     float q = d_x * d_x + d_y * d_y;
        //     Eigen::Vector3f z_pred = Eigen::Vector3f(std::sqrt(q), all_seaing_perception::mod_2pi(std::atan2(d_y, d_x) - m_state(2)), all_seaing_perception::angle_to_pi_range(d_theta));
        //     RCLCPP_INFO(this->get_logger(), "BANNER ORIENTATION: %lf, ROBOT ORIENTATION: %lf", m_state(3 + 2*m_num_obj + 3 * match[i] + 2), m_state(2));
        //     RCLCPP_INFO(this->get_logger(), "THETA ACTUAL: %lf, THETA PRED: %lf", z_actual(2), z_pred(2));
        //     if (abs(all_seaing_perception::angle_to_pi_range(z_actual(2)-z_pred(2))) > ((float)M_PI/2)){
        //         RCLCPP_INFO(this->get_logger(), "REJECTED");
        //         match[i] = -1;
        //     }
        // }
        if (match[i] == -1) {
            if (detected_banners[i]->label == -1){
                // don't create new obstacles when they have no label
                continue;
            }
            // increase object count and expand & initialize matrices
            m_num_banners++;
            // add object to tracked obstacles vector
            detected_banners[i]->plane_msg.id = m_obstacle_id++;
            m_tracked_banners.push_back(detected_banners[i]);

            gtsam::Key new_banner_key;
            new_banner_key = gtsam::Symbol('l', m_obstacle_id);
            m_tracked_banners.back()->node_key = new_banner_key;
            Eigen::Vector3d estimated_pose_vec = Eigen::Vector3d(m_robot_pos_mean(0), m_robot_pos_mean(1), m_robot_pos_mean(2)) +
                range * Eigen::Vector3d(std::cos(bearing + m_robot_pos_mean(2)),
                                        std::sin(bearing + m_robot_pos_mean(2)),
                                        0);
            estimated_pose_vec[0] = all_seaing_perception::mod_2pi(m_robot_pos_mean(2) + phi);
            gtsam::Pose2 new_banner_estimated_pose = gtsam::Pose2((gtsam::Vector)estimated_pose_vec.cast<double>());
            initialEstimate.insert(new_banner_key, new_banner_estimated_pose);
        }
        int tracked_id = match[i] >= 0 ? match[i] : m_num_banners - 1;

        // add the range bearing factor
        graph.add(all_seaing_perception::BearingRangePhiFactor(m_pose_keys.back(), m_tracked_banners[tracked_id]->node_key, bearing, range, phi, m_banner_meas_noise));

        // update data for matched obstacles (we'll update position after we update SLAM with all points)
        if (banner_label_indicator.count(detected_banners[i]->label) && (!banner_label_indicator[detected_banners[i]->label])){
            detected_banners[i]->label = m_tracked_banners[tracked_id]->label;
        }
        detected_banners[i]->plane_msg.id = m_tracked_banners[tracked_id]->plane_msg.id;
        detected_banners[i]->time_seen = m_tracked_banners[tracked_id]->time_seen;
        detected_banners[i]->last_dead = m_tracked_banners[tracked_id]->last_dead;
        detected_banners[i]->time_dead = m_tracked_banners[tracked_id]->time_dead;
        detected_banners[i]->is_dead = m_tracked_banners[tracked_id]->is_dead;
        detected_banners[i]->node_key = m_tracked_banners[tracked_id]->node_key;
        m_tracked_banners[tracked_id] = detected_banners[i];
    }

    m_isam2->update(graph, initialEstimate); // this runs the iSAM2 optimization step

    // update the means & covariances of everything
    this->update_estimates();
    
    this->update_maps();

    // Filter old obstacles
    std::unordered_set<int> to_keep_set;
    for (int tracked_id = 0; tracked_id < static_cast<int>(m_tracked_banners.size()); tracked_id++) {
        if (chosen_tracked.count(tracked_id)) {
            to_keep_set.insert(tracked_id);
            m_tracked_banners[tracked_id]->is_dead = false;
            m_tracked_banners[tracked_id]->time_dead = rclcpp::Duration(0,0);
            m_tracked_banners[tracked_id]->time_seen = m_local_header.stamp;
            continue;
        }
        // Dead
        if (m_tracked_banners[tracked_id]->is_dead) {
            // Was also dead before, add time dead
            m_tracked_banners[tracked_id]->time_dead =
                rclcpp::Time(m_local_header.stamp) -
                m_tracked_banners[tracked_id]->last_dead +
                m_tracked_banners[tracked_id]->time_dead;
            pcl::PointXYZ p0 = pcl::PointXYZ(m_robot_pos_mean(0), m_robot_pos_mean(1), 0);
            float dist = pcl::euclideanDistance(
                p0,
                pcl::PointXYZ(m_tracked_banners[tracked_id]->plane_msg.normal_ctr.position.x, 
                                m_tracked_banners[tracked_id]->plane_msg.normal_ctr.position.y, 
                                m_tracked_banners[tracked_id]->plane_msg.normal_ctr.position.z));
            if ((m_tracked_banners[tracked_id]->time_dead.seconds() > m_obstacle_drop_thresh) && (dist < m_range_drop_thresh)) {
                continue;
            }
        }
        m_tracked_banners[tracked_id]->is_dead = true;
        m_tracked_banners[tracked_id]->last_dead = m_local_header.stamp;
        to_keep_set.insert(tracked_id);
    }

    // Clear out duplicates
    // TODO: optimize to N^2 (now is N^3) by making a priority queue of pairs of indices based on minimum distance and popping if index already removed and removing if distance < threshold
    int ind_to_remove = 0;
    while(ind_to_remove != -1){
        ind_to_remove = -1;
        // find minimum distance obstacles and remove one seen earliest if under the duplicate threshold
        float min_dist = m_duplicate_thresh;
        for (int i : to_keep_set){
            for (int j : to_keep_set){
                if (i == j) continue;
                bool banner_number_same = false;
                if (banner_label_to_number.count(m_tracked_banners[i]->label) && banner_label_to_number.count(m_tracked_banners[j]->label)){
                    if (banner_label_to_number[m_tracked_banners[i]->label] != banner_label_to_number[m_tracked_banners[j]->label]){
                        continue;
                    }
                    banner_number_same = true;
                    // RCLCPP_INFO(this->get_logger(), "LABELS %d = %d", m_tracked_banners[i]->label, m_tracked_banners[j]->label);
                }else if (m_tracked_banners[i]->label != m_tracked_banners[j]->label){
                    continue;
                }
                float dist = pcl::euclideanDistance(
                    pcl::PointXYZ(m_tracked_banners[i]->plane_msg.normal_ctr.position.x, 
                                  m_tracked_banners[i]->plane_msg.normal_ctr.position.y, 
                                  m_tracked_banners[i]->plane_msg.normal_ctr.position.z),
                    pcl::PointXYZ(m_tracked_banners[j]->plane_msg.normal_ctr.position.x, 
                                  m_tracked_banners[j]->plane_msg.normal_ctr.position.y, 
                                  m_tracked_banners[j]->plane_msg.normal_ctr.position.z));
                if (dist < min_dist){
                    min_dist = dist;
                    // remove
                    // if (m_tracked_banners[i]->label == m_tracked_banners[j]->label){
                    if (banner_number_same){
                        if (banner_label_indicator[m_tracked_banners[i]->label] && (!banner_label_indicator[m_tracked_banners[j]->label])){
                            ind_to_remove = j;
                            continue;
                        }else if (banner_label_indicator[m_tracked_banners[j]->label] && (!banner_label_indicator[m_tracked_banners[i]->label])){
                            ind_to_remove = i;
                            continue;
                        }
                    }
                    ind_to_remove = m_tracked_banners[i]->cov.trace() < m_tracked_banners[j]->cov.trace()?j:i;
                    // }
                    // else{
                    //     ind_to_remove = (m_tracked_banners[i]->time_seen < m_tracked_banners[j]->time_seen)?i:j;
                    // }
                }
            }
        }
        if (ind_to_remove != -1){
            to_keep_set.erase(ind_to_remove);
        }
    }

    std::vector<int> to_remove;
    std::vector<int> to_keep;
    for (int i = 0; i < m_tracked_banners.size(); i++){
        if(to_keep_set.count(i)){
            to_keep.push_back(i);
        }else{
            to_remove.push_back(i);
        }
    }

    // update vectors & matrices
    if (!to_remove.empty()) {
        std::vector<std::shared_ptr<all_seaing_perception::Banner>> new_obj;

        for (int i : to_keep) {
            new_obj.push_back(m_tracked_banners[i]);
        }

        // can't remove from GTSAM factor graph because it affects the belief
        // TODO apply variable elimination (marginalize then remove from the graph), see how to do it in GTSAM

        m_tracked_banners = new_obj;
        m_num_banners = to_keep.size();
    }

    m_trace.push_back(std::make_tuple(m_robot_pos_mean(0), m_robot_pos_mean(1), rclcpp::Time(m_local_header.stamp).seconds()));

    this->publish_maps();

    if(m_got_nav){
        this->visualize_predictions();
    }
    if(m_got_nav && m_got_odom){
        this->publish_slam();
    }
}

FactorGraphSLAM::~FactorGraphSLAM() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FactorGraphSLAM>());
    rclcpp::shutdown();
    return 0;
}
