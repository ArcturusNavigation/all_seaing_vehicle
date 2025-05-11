#include "all_seaing_perception/object_tracking_map.hpp"

cv::Point2d custom_project(image_geometry::PinholeCameraModel cmodel, const cv::Point3d& xyz){
    cv::Point2d uv_rect;
    uv_rect.x = (cmodel.fx()*xyz.x + cmodel.Tx()) / xyz.z + cmodel.cx();
    uv_rect.y = (cmodel.fy()*xyz.y + cmodel.Ty()) / xyz.z + cmodel.cy();
    return uv_rect;
}

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map") {
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("slam_frame_id", "slam_map");
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("range_uncertainty", 1.0);
    this->declare_parameter<double>("bearing_uncertainty", 1.0);
    this->declare_parameter<double>("motion_gps_xy_noise", 1.0);
    this->declare_parameter<double>("motion_gps_theta_noise", 1.0);
    this->declare_parameter<double>("motion_imu_xy_noise", 1.0);
    this->declare_parameter<double>("motion_imu_theta_noise", 1.0);
    this->declare_parameter<double>("update_gps_xy_uncertainty", 1.0);
    this->declare_parameter<double>("update_odom_theta_uncertainty", 1.0);
    this->declare_parameter<double>("new_object_slam_threshold", 1.0);
    this->declare_parameter<double>("init_xy_noise", 1.0);
    this->declare_parameter<double>("init_theta_noise", 1.0);
    this->declare_parameter<double>("init_new_cov", 10.0);
    this->declare_parameter<bool>("track_robot", true);
    this->declare_parameter<bool>("imu_predict", true);
    this->declare_parameter<bool>("gps_update", true);
    this->declare_parameter<double>("normalize_drop_dist", 1.0);
    this->declare_parameter<double>("odom_refresh_rate", 40);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_slam_frame_id = this->get_parameter("slam_frame_id").as_string();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_range_std = this->get_parameter("range_uncertainty").as_double();
    m_bearing_std = this->get_parameter("bearing_uncertainty").as_double();
    m_gps_xy_noise = this->get_parameter("motion_gps_xy_noise").as_double();
    m_gps_theta_noise = this->get_parameter("motion_gps_theta_noise").as_double();
    m_imu_xy_noise = this->get_parameter("motion_imu_xy_noise").as_double();
    m_imu_theta_noise = this->get_parameter("motion_imu_theta_noise").as_double();
    m_update_gps_xy_uncertainty = this->get_parameter("update_gps_xy_uncertainty").as_double();
    m_update_odom_theta_uncertainty = this->get_parameter("update_odom_theta_uncertainty").as_double();
    m_new_obj_slam_thres = this->get_parameter("new_object_slam_threshold").as_double();
    m_init_xy_noise = this->get_parameter("init_xy_noise").as_double();
    m_init_theta_noise = this->get_parameter("init_theta_noise").as_double();
    m_init_new_cov = this->get_parameter("init_new_cov").as_double();
    m_track_robot = this->get_parameter("track_robot").as_bool();
    m_imu_predict = this->get_parameter("imu_predict").as_bool();
    m_gps_update = this->get_parameter("gps_update").as_bool();
    
    m_normalize_drop_dist = this->get_parameter("normalize_drop_dist").as_double();
    m_odom_refresh_rate = this->get_parameter("odom_refresh_rate").as_double();

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    this->declare_parameter<bool>("check_fov", true);
    m_check_fov = this->get_parameter("check_fov").as_bool();

    this->declare_parameter<bool>("direct_tf", true);
    m_direct_tf = this->get_parameter("direct_tf").as_bool();

    this->declare_parameter<bool>("rotate_odom", false);
    m_rotate_odom = this->get_parameter("rotate_odom").as_bool();

    this->declare_parameter<bool>("normalize_drop_thresh", false);
    m_normalize_drop_thresh = this->get_parameter("normalize_drop_thresh").as_bool();

    this->declare_parameter<bool>("include_odom_theta", true);
    m_include_odom_theta = this->get_parameter("include_odom_theta").as_bool();

    // Initialize navigation & odometry variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_z = 0;
    m_nav_vx = 0;
    m_nav_vy = 0;
    m_nav_vz = 0;
    m_nav_omega = 0;
    m_nav_heading = 0;

    // Initialize publishers and subscribers
    m_untracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_untracked", 10);
    m_tracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_tracked", 10);
    m_map_cov_viz_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "obstacle_map/map_cov_viz", 10);
    m_object_sub =
        this->create_subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>(
            "refined_object_point_clouds_segments", 10,
            std::bind(&ObjectTrackingMap::object_track_map_publish, this, std::placeholders::_1));
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_topic", 10,
        std::bind(&ObjectTrackingMap::intrinsics_cb, this, std::placeholders::_1));

    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // update odometry based on transforms on timer click
    odom_timer = this->create_wall_timer(
    std::chrono::duration<float>(((float)1.0)/m_odom_refresh_rate), std::bind(&ObjectTrackingMap::odom_callback, this));

    if(m_track_robot){
        // publish computed global robot position as a transform slam_map->map and a message with the pose of the robot wrt slam_map
        m_slam_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            "odometry/tracked", 10);
        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // update odometry based on IMU data by subscribing to the odometry message
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10,
            std::bind(&ObjectTrackingMap::odom_msg_callback, this, std::placeholders::_1));
    }

    m_first_state = true;
    m_got_local_frame = false;
    m_got_nav = false;
    m_got_odom = false;
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

void ObjectTrackingMap::publish_slam(){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m_last_odom_msg.header.stamp;
    // t.header.stamp = this->get_clock()->now();
    if(m_direct_tf){
        // publish the transform from the local frame (base_link) to slam_map
        // (slam_map is a child of base_link to not interfere with the ekf localization node map)
        // t.header.frame_id = m_slam_frame_id;
        // t.child_frame_id = m_local_frame_id;
        // t.transform.translation.x = m_state(0);
        // t.transform.translation.y = m_state(1);
        // t.transform.translation.z = m_nav_z;

        // tf2::Quaternion q;
        // q.setRPY(0, 0, m_state(2));
        // t.transform.rotation = tf2::toMsg(q);
        t.header.frame_id = m_local_frame_id;
        t.child_frame_id = m_slam_frame_id;
        float inv_x, inv_y, inv_theta;
        std::tie(inv_x, inv_y, inv_theta) = all_seaing_perception::compute_transform_from_to(m_state(0), m_state(1), m_state(2), 0, 0, 0);
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
        std::tie(shift_x, shift_y, shift_theta) =all_seaing_perception::compose_transforms(std::make_tuple(m_state(0), m_state(1), m_state(2)),all_seaing_perception:: compute_transform_from_to(m_nav_x, m_nav_y, m_nav_heading, 0, 0, 0));
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
    odom_msg.pose.pose.position.x = m_state(0);
    odom_msg.pose.pose.position.y = m_state(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, m_state(2));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    m_slam_pub->publish(odom_msg);
}

void ObjectTrackingMap::odom_msg_callback(const nav_msgs::msg::Odometry &msg){
    // !!! THOSE ARE RELATIVE TO THE ROBOT AND DEPENDENT ON ITS HEADING, USE THE CORRECT MOTION MODEL
    if(!m_rotate_odom){
        m_nav_vx = msg.twist.twist.linear.x;
        m_nav_vy = msg.twist.twist.linear.y;   
    }else{
        // Pixhawk rotated to the left, facing up, so need to rotate the acceleration vector accordingly
        m_nav_vx = -msg.twist.twist.linear.y;
        m_nav_vy = msg.twist.twist.linear.x;
        // m_nav_vx = msg.twist.twist.linear.x;
        // m_nav_vy = msg.twist.twist.linear.y;
    }
    m_nav_vz = msg.twist.twist.linear.z;
    m_nav_omega = msg.twist.twist.angular.z;
    m_last_odom_msg = msg;

    rclcpp::Time curr_odom_time = rclcpp::Time(msg.header.stamp);

    if(!m_got_odom){
        m_last_odom_time = curr_odom_time;
        m_got_odom = true;
        return;
    }

    float dt = (curr_odom_time - m_last_odom_time).seconds();
    m_last_odom_time = curr_odom_time;

    // RCLCPP_INFO(this->get_logger(), "IMU MESSAGE ODOM: (%lf, %lf), %lf", m_nav_vx, m_nav_vy, m_nav_omega);

    if (!m_got_nav || !m_track_robot || !m_imu_predict || m_first_state) return;

    Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3, 3 + 2 * m_num_obj);
    F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();

    /*
    IMU MOTION PREDICTION MODEL:
    (x_old, y_old, theta_old) -> (x_old+cos(theta_old)*v_x*dt-sin(theta_old)*v_y*dt, y_old+sin(theta_old)*d_x*dt+cos(theta_old)*v_y*dt, theta_old+omega*dt)
    mot_grad: rows -> components of final state (actually difference with old state), columns -> components of old state wrt to which the gradient is taken
    */

    Eigen::Vector3f mot_const = Eigen::Vector3f(cos(m_state(2))*m_nav_vx*dt-sin(m_state(2))*m_nav_vy*dt, sin(m_state(2))*m_nav_vx*dt+cos(m_state(2))*m_nav_vy*dt, m_nav_omega*dt);
    // gradient - identity
    Eigen::Matrix3f mot_grad{
        {0, 0, -sin(m_state(2))*m_nav_vx*dt-cos(m_state(2))*m_nav_vy*dt},
        {0, 0, cos(m_state(2))*m_nav_vx*dt-sin(m_state(2))*m_nav_vy*dt},
        {0, 0, 0},
    };

    m_state += F.transpose() * mot_const;

    // RCLCPP_INFO(this->get_logger(), "ROBOT PREDICTED POSE AFTER IMU UPDATE: (%lf, %lf), %lf", m_state(0), m_state(1), m_state(2));
    Eigen::MatrixXf G = Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) +
                        F.transpose() * mot_grad * F;
    // add a consistent amount of noise based on how much the robot moved since the last time
    // Eigen::Matrix3f motion_noise{
    //     {m_xy_noise * abs(mot_const[0]), 0, 0},
    //     {0, m_xy_noise * abs(mot_const[1]), 0},
    //     {0, 0, m_theta_noise * abs(mot_const[2])},
    // };
    Eigen::Matrix3f motion_noise{
        {m_imu_xy_noise, 0, 0},
        {0, m_imu_xy_noise, 0},
        {0, 0, m_imu_theta_noise},
    };
    m_cov = G * m_cov * G.transpose() + F.transpose() * motion_noise * F;

    if (m_track_robot) {
        m_trace.push_back(std::make_pair(m_state(0), m_state(1)));
    }

    this->publish_maps();
    
    // to see the robot position prediction mean & uncertainty
    if(m_got_nav){
        this->visualize_predictions();
    }
    // publish robot pose predictions (here and after measurement update) as transforms from map to lidar/camera frame, possibly also as a message
    // it should ultimately be a source of odometry that can be used by other nodes along with the global map
    if(m_track_robot && m_got_nav && m_got_odom){
        publish_slam();
    }
}

void ObjectTrackingMap::odom_callback() {
    if(!m_got_local_frame) return;

    //update odometry transforms
    //TODO: add a flag for each one that says if they succedeed, to know to continue or not
    // RCLCPP_INFO(this->get_logger(), "ODOM CALLBACK");
    m_map_base_link_tf = all_seaing_perception::get_tf(m_tf_buffer, m_global_frame_id, m_local_frame_id);
    m_base_link_map_tf = all_seaing_perception::get_tf(m_tf_buffer, m_local_frame_id, m_global_frame_id);

    m_nav_z = m_map_base_link_tf.transform.translation.z;
    tf2::Quaternion quat;
    tf2::fromMsg(m_map_base_link_tf.transform.rotation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    m_nav_heading = y;

    m_got_nav = true;

    // to ignore gps TFs that are the same thing 5000 times a second and cause SLAM to lock into the GPS
    // RCLCPP_INFO(this->get_logger(), "COMPARE: (%lf, %lf), (%lf, %lf)", m_nav_x, m_nav_y, m_map_base_link_tf.transform.translation.x, m_map_base_link_tf.transform.translation.y);
    if((!m_first_state) && (m_nav_x == m_map_base_link_tf.transform.translation.x) && (m_nav_y == m_map_base_link_tf.transform.translation.y)) return;

    // RCLCPP_INFO(this->get_logger(), "GOT ODOM");
    m_nav_x = m_map_base_link_tf.transform.translation.x;
    m_nav_y = m_map_base_link_tf.transform.translation.y;
    // RCLCPP_INFO(this->get_logger(), "COMPARE NEW: (%lf, %lf), (%lf, %lf)", m_nav_x, m_nav_y, m_map_base_link_tf.transform.translation.x, m_map_base_link_tf.transform.translation.y);

    // RCLCPP_INFO(this->get_logger(), "NEW GPS: (%lf, %lf)", m_nav_x, m_nav_y);

    if(!m_track_robot) return;

    if (m_first_state) {
        // initialize mean and cov robot pose
        m_state = Eigen::Vector3f(m_nav_x, m_nav_y, m_nav_heading);
        Eigen::Matrix3f init_pose_noise{
            {m_init_xy_noise, 0, 0},
            {0, m_init_xy_noise, 0},
            {0, 0, m_init_theta_noise},
        };
        m_cov = init_pose_noise;
        // m_cov = Eigen::Matrix3f::Zero();
        m_first_state = false;
        // m_last_odom_time = rclcpp::Time(msg.header.stamp);
        return;
    }

    if (!m_imu_predict) {

        // rclcpp::Time curr_odom_time = rclcpp::Time(msg.header.stamp);
        // float dt = (curr_odom_time - m_last_odom_time).seconds();
        // m_last_odom_time = curr_odom_time;
        // m_nav_omega = (m_nav_heading - m_state[2]) / dt;

        // RCLCPP_INFO(this->get_logger(), "ROBOT ODOM DATA: (%lf, %lf, %lf), pub omega: %lf, dt:
        // %lf, comp omega: %lf", m_nav_x, m_nav_y, m_nav_heading, msg.twist.twist.angular.z, dt,
        // (m_nav_heading-m_state[2])/dt);

        Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3, 3 + 2 * m_num_obj);
        F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
        
        /*
        GPS MOTION PREDICTION MODEL:
        (x_old, y_old, theta_old) -> (x_old+v_comp_x*dt, y_old+v_comp_y*dt, theta_old+omega_comp*dt)
        -->gradient is identity matrix (that way we keep the correlations with the objects, if we
        set it to the new values it would be zero and delete them) remember that mot_grad is the
        gradients minus the identity matrix, so with this model it is the zero matrix
        */

        Eigen::Vector3f mot_const = Eigen::Vector3f(m_nav_x - m_state[0], m_nav_y - m_state[1], m_nav_heading - m_state[2]);
        Eigen::Matrix3f mot_grad = Eigen::Matrix3f::Zero();

        m_state += F.transpose() * mot_const;
        Eigen::MatrixXf G = Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) +
                            F.transpose() * mot_grad * F;
        // add a consistent amount of noise based on how much the robot moved since the last time
        Eigen::Matrix3f motion_noise{
            {m_gps_xy_noise * abs(mot_const[0]), 0, 0},
            {0, m_gps_xy_noise * abs(mot_const[1]), 0},
            {0, 0, m_gps_theta_noise * abs(mot_const[2])},
        };
        m_cov = G * m_cov * G.transpose() + F.transpose() * motion_noise * F;
    }else if (m_gps_update){
        if(!m_include_odom_theta){
            // GPS measurement update model is just a gaussian centered at the predicted (x,y) position of the robot, with some noise
            Eigen::Matrix2f Q{
                {m_update_gps_xy_uncertainty, 0},
                {0, m_update_gps_xy_uncertainty},
            };
            Eigen::Vector2f xy_actual(m_nav_x, m_nav_y);
            Eigen::Vector2f xy_pred(m_state(0), m_state(1));
            //gradient of measurement update model, identity since it's centered at the initial state
            Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2, 3 + 2 * m_num_obj);
            H.topLeftCorner(2, 2) = Eigen::Matrix2f::Identity();
            // Eigen::Matrix2f h = Eigen::Matrix2f::Identity();
            // Eigen::MatrixXf F = Eigen::MatrixXf::Zero(2, 3 + 2 * m_num_obj);
            // F.topLeftCorner(2, 2) = Eigen::Matrix2f::Identity();
            // Eigen::MatrixXf H = h * F;
            Eigen::MatrixXf K = m_cov * H.transpose() * (H * m_cov * H.transpose() + Q).inverse();
            m_state += K * (xy_actual - xy_pred);
            m_cov =
                (Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) - K * H) * m_cov;   
        }else{
            // Include theta, since that's provided by the IMU compass usually
            Eigen::Matrix3f Q{
                {m_update_gps_xy_uncertainty, 0, 0},
                {0, m_update_gps_xy_uncertainty, 0},
                {0, 0, m_update_odom_theta_uncertainty},
            };
            Eigen::Vector3f xyth_actual(m_nav_x, m_nav_y, m_nav_heading);
            Eigen::Vector3f xyth_pred(m_state(0), m_state(1), m_state(2));
            //gradient of measurement update model, identity since it's centered at the initial state
            Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 3 + 2 * m_num_obj);
            H.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
            Eigen::MatrixXf K = m_cov * H.transpose() * (H * m_cov * H.transpose() + Q).inverse();
            m_state += K * (xyth_actual - xyth_pred);
            m_cov =
                (Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) - K * H) * m_cov;
        }
    }

    if (m_track_robot) {
        m_trace.push_back(std::make_pair(m_state(0), m_state(1)));
    }

    this->publish_maps();

    if(m_got_nav){
        this->visualize_predictions();
    }

    if(m_track_robot && m_got_nav && m_got_odom){
        publish_slam();
    }

    this->update_maps();
    this->publish_maps();
}

void ObjectTrackingMap::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    // RCLCPP_INFO(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

template <typename T>
T ObjectTrackingMap::convert_to_global(T point, bool untracked) {
    T new_point = point; // to keep color-related data
    geometry_msgs::msg::Point lc_pt_msg;
    lc_pt_msg.x = point.x;
    lc_pt_msg.y = point.y;
    lc_pt_msg.z = point.z;
    geometry_msgs::msg::Point gb_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(lc_pt_msg, gb_pt_msg, m_base_link_map_tf);
    new_point.x = gb_pt_msg.x;
    new_point.y = gb_pt_msg.y;
    new_point.z = gb_pt_msg.z;
    T act_point = new_point;
    if(m_track_robot && (!untracked)){
        // point initially in map frame
        // want slam_map->map (then will compose it with slam->point)
        // (slam_map->robot)@(robot->map) = (slam_map->robot)@inv(map->robot)
        std::tuple<double, double, double> slam_to_map_transform =all_seaing_perception::compose_transforms(std::make_tuple(m_state(0), m_state(1), m_state(2)),all_seaing_perception:: compute_transform_from_to(m_nav_x, m_nav_y, m_nav_heading, 0, 0, 0));
        double th; //uselesss
        std::tie(act_point.x, act_point.y, th) =all_seaing_perception::compose_transforms(slam_to_map_transform, std::make_tuple(new_point.x, new_point.y, 0));
    }
    return act_point;
}

template <typename T>
T ObjectTrackingMap::convert_to_local(T point, bool untracked) {
    T new_point = point; // to keep color-related data
    T act_point = point;
    if(m_track_robot && (!untracked)){
        // point initially in slam_map frame
        // want map->slam_map (then will compose it with slam->point)
        // (map->robot)@(robot->slam_map) = (map->robot)@inv(slam_map->robot)
        std::tuple<double, double, double> map_to_slam_transform =all_seaing_perception::compose_transforms(std::make_tuple(m_nav_x, m_nav_y, m_nav_heading),all_seaing_perception:: compute_transform_from_to(m_state(0), m_state(1), m_state(2), 0, 0, 0));
        double th; //uselesss
        std::tie(act_point.x, act_point.y, th) =all_seaing_perception::compose_transforms(map_to_slam_transform, std::make_tuple(new_point.x, new_point.y, 0));
    }
    geometry_msgs::msg::Point gb_pt_msg;
    gb_pt_msg.x = act_point.x;
    gb_pt_msg.y = act_point.y;
    gb_pt_msg.z = act_point.z;
    geometry_msgs::msg::Point lc_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(gb_pt_msg, lc_pt_msg, m_map_base_link_tf);
    new_point.x = lc_pt_msg.x;
    new_point.y = lc_pt_msg.y;
    new_point.z = lc_pt_msg.z;
    return new_point;
}

void ObjectTrackingMap::update_maps(){
    // Update all (new and old, since they also have correlation with each other) objects global
    // positions (including the cloud, and then the local points respectively)
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        pcl::PointXYZ upd_glob_centr = m_tracked_obstacles[i]->global_centroid;
        if (m_track_robot) {
            upd_glob_centr.x = m_state[3 + 2 * i];
            upd_glob_centr.y =
                m_state[3 + 2 * i + 1]; // to not lose the z coordinate, it's useful for checking if
                                        // the objects should be in the camera frame
        } else {
            upd_glob_centr.x = m_tracked_obstacles[i]->mean_pred[0];
            upd_glob_centr.y = m_tracked_obstacles[i]->mean_pred[1];
        }
        // map->point = (map->centroid)@(centroid->point)
        float useless_theta;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        for (pcl::PointXYZHSV &global_pt : m_tracked_obstacles[i]->global_pcloud_ptr->points){
            pcl::PointXYZHSV upd_global_pt = global_pt;
            std::tuple<double, double, double> centroid_to_point = all_seaing_perception::compute_transform_from_to(m_tracked_obstacles[i]->global_centroid.x, m_tracked_obstacles[i]->global_centroid.y, 0, global_pt.x, global_pt.y, 0);
            std::tie(upd_global_pt.x, upd_global_pt.y, useless_theta) = all_seaing_perception::compose_transforms(std::make_tuple(upd_glob_centr.x, upd_glob_centr.y, 0), centroid_to_point);
            global_pt = upd_global_pt;
            upd_local_obj_pcloud->push_back(
                this->convert_to_local(global_pt));
        }

        m_tracked_obstacles[i]->global_centroid = upd_glob_centr;
        m_tracked_obstacles[i]->local_centroid = this->convert_to_local(upd_glob_centr);
    }
}

void ObjectTrackingMap::publish_maps(){
    // Publish map with tracked obstacles
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> tracked_obs;
    std::vector<int> tracked_labels;
    for (std::shared_ptr<all_seaing_perception::ObjectCloud> t_ob : m_tracked_obstacles) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_tracked_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (pcl::PointXYZHSV pt : t_ob->local_pcloud_ptr->points) {
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            local_tracked_cloud->push_back(i_pt);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr global_tracked_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (pcl::PointXYZHSV pt : t_ob->global_pcloud_ptr->points) {
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            global_tracked_cloud->push_back(i_pt);
        }

        std::shared_ptr<all_seaing_perception::Obstacle> tracked_ob(
            new all_seaing_perception::Obstacle(m_local_header, m_global_header, local_tracked_cloud, global_tracked_cloud,
                                                t_ob->id));
        tracked_labels.push_back(t_ob->label);
        tracked_obs.push_back(tracked_ob);
    }
    all_seaing_perception::publish_map(m_local_header, m_global_header, "tracked", true, tracked_obs, m_tracked_map_pub,
                    tracked_labels);
}

void ObjectTrackingMap::visualize_predictions() {
    // RCLCPP_INFO(this->get_logger(), "NUMBER OF OBJECTS: %d", m_num_obj);
    // RCLCPP_INFO(this->get_logger(), "STATE SIZE: %d", m_state.rows());
    // RCLCPP_INFO(this->get_logger(), "COV SIZE: %dx%d", m_cov.rows(), m_cov.cols());

    // delete previous markers
    visualization_msgs::msg::MarkerArray delete_arr;
    visualization_msgs::msg::Marker delete_mark;
    delete_mark.id = 0;
    delete_mark.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_arr.markers.push_back(delete_mark);
    m_map_cov_viz_pub->publish(delete_arr);

    visualization_msgs::msg::MarkerArray ellipse_arr;
    // robot pose prediction
    if (m_track_robot) {
        Eigen::Vector2f robot_mean = m_state.segment(0, 2);
        Eigen::Matrix2f robot_cov = m_cov.block(0, 0, 2, 2);
        // RCLCPP_INFO(this->get_logger(), "ROBOT PREDICTED POSE: (%lf, %lf)", robot_mean(0), robot_mean(1));
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(robot_cov);
        double a_x = eigen_solver.eigenvalues()(0);
        double a_y = eigen_solver.eigenvalues()(1);
        // RCLCPP_INFO(this->get_logger(), "ROBOT COVARIANCE AXES LENGTHS: (%lf, %lf)", a_x, a_y);
        Eigen::Vector2f axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2f axis_y = eigen_solver.eigenvectors().col(1);
        tf2::Matrix3x3 rot_mat(axis_x(0), axis_y(0), 0, axis_x(1), axis_y(1), 0, 0, 0, 1);
        tf2::Quaternion quat_rot;
        rot_mat.getRotation(quat_rot);
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;
        ellipse.pose.position.x = robot_mean(0);
        ellipse.pose.position.y = robot_mean(1);
        ellipse.pose.position.z = 0;
        ellipse.pose.orientation = tf2::toMsg(quat_rot);
        // if(m_is_sim){
        //     ellipse.pose.position.x = m_nav_x; // to overlay the map on top of the transform (GPS), especially in sim
        //     ellipse.pose.position.y = m_nav_y;
        // }
        ellipse.scale.x = sqrt(a_x);
        ellipse.scale.y = sqrt(a_y);
        ellipse.scale.z = 0.5;
        ellipse.color.a = 0.5;
        ellipse.color.g = 1;
        ellipse.header = m_global_header;
        ellipse.header.frame_id = m_slam_frame_id;
        ellipse.id = m_num_obj + 1;
        ellipse_arr.markers.push_back(ellipse);

        visualization_msgs::msg::Marker angle_marker;
        angle_marker.type = visualization_msgs::msg::Marker::ARROW;
        angle_marker.pose.position.x = robot_mean(0);
        angle_marker.pose.position.y = robot_mean(1);
        angle_marker.pose.position.z = 0;
        tf2::Quaternion angle_quat;
        angle_quat.setRPY(0, 0, m_state(2));
        angle_marker.pose.orientation = tf2::toMsg(angle_quat);
        // if(m_is_sim){
        //     ellipse.pose.position.x = m_nav_x;
        //     ellipse.pose.position.y = m_nav_y;
        //     tf2::Quaternion nav_quat;
        //     nav_quat.setRPY(0,0,m_nav_heading);
        //     ellipse.pose.orientation = tf2::toMsg(nav_quat);
        // }
        angle_marker.scale.x = m_cov(2, 2);
        angle_marker.scale.y = 0.2;
        angle_marker.scale.z = 0.2;
        angle_marker.color.a = 1;
        angle_marker.header = m_global_header;
        angle_marker.header.frame_id = m_slam_frame_id;
        angle_marker.id = m_num_obj + 2;
        ellipse_arr.markers.push_back(angle_marker);
    }

    // obstacle predictions
    for (int i = 0; i < m_num_obj; i++) {
        Eigen::Vector2f obj_mean;
        Eigen::Matrix2f obj_cov;
        if (m_track_robot) {
            obj_mean = m_state.segment(
                3 + 2 * i,
                2); // z coord is the label (will visualize the variance of that too to identify
                    // possible bugs in its update, because it should not be changed)
            obj_cov = m_cov.block(3 + 2 * i, 3 + 2 * i, 2, 2);
        } else {
            obj_mean = m_tracked_obstacles[i]->mean_pred;
            obj_cov = m_tracked_obstacles[i]->cov;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(obj_cov);
        double a_x = eigen_solver.eigenvalues()(0);
        double a_y = eigen_solver.eigenvalues()(1);
        // RCLCPP_INFO(this->get_logger(), "OBJECT %d COVARIANCE AXES LENGTHS: (%lf, %lf)", i, a_x, a_y);
        Eigen::Vector2f axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2f axis_y = eigen_solver.eigenvectors().col(1);
        double cov_scale = m_new_obj_slam_thres; // to visualize the threshold where new obstacles are added
        tf2::Matrix3x3 rot_mat(axis_x(0), axis_y(0), 0, axis_x(1), axis_y(1), 0, 0, 0, 1);
        tf2::Quaternion quat_rot;
        rot_mat.getRotation(quat_rot);
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;

        // if (m_track_robot && m_is_sim){
        //     double final_x, final_y, final_theta;
        //     double r,p,init_theta;
        //     rot_mat.getRPY(r,p,init_theta);
        //     std::tie(final_x, final_y, final_theta) = ObjectTrackingMap::apply_transform_from_to(obj_mean(0), obj_mean(1), init_theta, m_state(0), m_state(1), m_state(2), m_nav_x, m_nav_y, m_nav_heading);
        //     tf2::Quaternion final_quat_rot;
        //     final_quat_rot.setRPY(0,0,final_theta);
        //     // RCLCPP_INFO(this->get_logger(), "ROBOT: (%lf, %lf, %lf) -> (%lf, %lf, %lf), OBJECT: (%lf, %lf, %lf) -> (%lf, %lf, %lf)",m_state(0), m_state(1), m_state(2), m_nav_x, m_nav_y, m_nav_heading, obj_mean(0), obj_mean(1), init_theta, final_x, final_y, final_theta);
        //     ellipse.pose.position.x = final_x;
        //     ellipse.pose.position.y = final_y;
        //     ellipse.pose.orientation = tf2::toMsg(final_quat_rot);
        // }else{
        ellipse.pose.position.x = obj_mean(0);
        ellipse.pose.position.y = obj_mean(1);
        ellipse.pose.orientation = tf2::toMsg(quat_rot);
        // }
        
        ellipse.pose.position.z = 0;
        ellipse.scale.x = cov_scale * sqrt(a_x);
        ellipse.scale.y = cov_scale * sqrt(a_y);
        ellipse.scale.z = 1;
        ellipse.color.a = 0.2;
        ellipse.color.r = 1;
        ellipse.header = m_global_header;
        if(m_track_robot){
            ellipse.header.frame_id = m_slam_frame_id;
        }
        ellipse.id = i;
        ellipse_arr.markers.push_back(ellipse);
    }

    visualization_msgs::msg::Marker trace;
    trace.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trace.header = m_global_header;
    trace.id = m_num_obj+3;
    trace.scale.x = 0.1;
    trace.scale.y = 0.1;
    trace.color.a = 1;
    trace.color.r = 1;
    trace.color.g = 1;
    for (std::pair<float, float> p : m_trace){
        geometry_msgs::msg::Point pt = geometry_msgs::msg::Point();
        pt.x = p.first;
        pt.y = p.second;
        trace.points.push_back(pt);
    }
    ellipse_arr.markers.push_back(trace);

    m_map_cov_viz_pub->publish(ellipse_arr);
}

void ObjectTrackingMap::object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg){
    // if(msg->objects.size()==0) return;
    // RCLCPP_INFO(this->get_logger(), "GOT DATA");

    // Set up headers and transforms
    // m_local_header = msg->objects[0].cloud.header;
    m_local_header = msg->header;
    m_global_header.frame_id = m_track_robot? m_slam_frame_id : m_global_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    std_msgs::msg::Header m_global_untracked_header = m_global_header;
    m_global_untracked_header.frame_id = m_global_frame_id;
    m_local_frame_id = m_local_header.frame_id;
    m_got_local_frame = true;

    // RCLCPP_INFO(this->get_logger(), "BEFORE GETTING ODOMETRY TF");
    m_base_link_map_tf = all_seaing_perception::get_tf(m_tf_buffer, m_global_frame_id, m_local_frame_id);
    m_map_base_link_tf = all_seaing_perception::get_tf(m_tf_buffer, m_local_frame_id, m_global_frame_id);
    // RCLCPP_INFO(this->get_logger(), "LOCAL FRAME: %s, GLOBAL FRAME: %s", m_local_frame_id.c_str(), m_global_frame_id.c_str());
    // RCLCPP_INFO(this->get_logger(), "GOT ODOMETRY TF");

    if(m_track_robot && m_first_state) return;

    std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud>> detected_obstacles;
    for (all_seaing_interfaces::msg::LabeledObjectPointCloud obj : msg->objects) {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(obj.cloud, *local_obj_pcloud);
        for (pcl::PointXYZHSV &pt : local_obj_pcloud->points) {
            pcl::PointXYZHSV global_pt =
                this->convert_to_global(pt, true);
            global_obj_pcloud->push_back(global_pt);
        }
        // pcl::transformPointCloud(*local_obj_pcloud, *global_obj_pcloud, m_base_link_map_tf);
        std::shared_ptr<all_seaing_perception::ObjectCloud> obj_cloud(
            new all_seaing_perception::ObjectCloud(obj.time, obj.label, local_obj_pcloud, global_obj_pcloud));
        detected_obstacles.push_back(obj_cloud);
    }
    // Make and publish untracked map
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> untracked_obs;
    std::vector<int> untracked_labels;
    for (std::shared_ptr<all_seaing_perception::ObjectCloud> det_obs : detected_obstacles) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (pcl::PointXYZHSV pt : det_obs->local_pcloud_ptr->points) {
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            raw_cloud->push_back(i_pt);
        }
        std::vector<int> ind(raw_cloud->size());
        std::iota(std::begin(ind), std::end(ind), 0); 
        std::shared_ptr<all_seaing_perception::Obstacle> untracked_ob(
            new all_seaing_perception::Obstacle(m_local_header, m_global_untracked_header, raw_cloud, ind,
                                                m_obstacle_id++, m_base_link_map_tf));
        untracked_labels.push_back(det_obs->label);
        untracked_obs.push_back(untracked_ob);
    }
    all_seaing_perception::publish_map(m_local_header, m_global_untracked_header, "untracked", true, untracked_obs, m_untracked_map_pub,
                    untracked_labels);

    // EKF SLAM ("Probabilistic Robotics", Seb. Thrun, inspired implementation)

    Eigen::Matrix<float, 2, 2> Q{
        {m_range_std, 0},
        {0, m_bearing_std},
    };
    std::vector<std::vector<float>> p;
    // RCLCPP_INFO(this->get_logger(), "COMPUTE WITH UNKNOWN CORRESPONDENCE");
    for (std::shared_ptr<all_seaing_perception::ObjectCloud> det_obs : detected_obstacles) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) =
            all_seaing_perception::local_to_range_bearing_signature(det_obs->local_centroid, det_obs->label);
        p.push_back(std::vector<float>());
        Eigen::Vector2f z_pred;
        Eigen::MatrixXf Psi;
        for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
            if (m_track_robot) {
                float d_x = m_state(3 + 2 * tracked_id) - m_state(0);
                float d_y = m_state(3 + 2 * tracked_id + 1) - m_state(1);
                float q = d_x * d_x + d_y * d_y;
                z_pred = Eigen::Vector2f(std::sqrt(q), std::atan2(d_y, d_x) - m_state(2));
                Eigen::MatrixXf F = Eigen::MatrixXf::Zero(5, 3 + 2 * m_num_obj);
                F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
                F.block(3, 3 + 2 * tracked_id, 2, 2) = Eigen::Matrix2f::Identity();
                Eigen::Matrix<float, 2, 5> h{
                    {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x,
                     std::sqrt(q) * d_y},
                    {d_y, -d_x, -1, -d_y, d_x},
                };
                // Do not store vectors, since they don't compute covariance with other obstacles in
                // the same detection batch
                Eigen::MatrixXf H = h * F / q;

                Psi = H * m_cov * H.transpose() + Q;
            } else {
                float d_x = m_tracked_obstacles[tracked_id]->mean_pred[0] - m_nav_x;
                float d_y = m_tracked_obstacles[tracked_id]->mean_pred[1] - m_nav_y;
                float q = d_x * d_x + d_y * d_y;
                z_pred = Eigen::Vector2f(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
                // Eigen::MatrixXf F = Eigen::MatrixXf::Zero(2, 2*m_num_obj);
                // F.block(0, 2*tracked_id, 2, 2) = Eigen::MatrixXf::Identity(2,2);
                Eigen::Matrix<float, 2, 2> h{
                    {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                    {-d_y, d_x},
                };
                // Do not store vectors, since they don't compute covariance with other obstacles in
                // the same detection batch
                //  Eigen::MatrixXf H = h*F/q;
                Eigen::MatrixXf H = h / q;
                Psi = H * m_tracked_obstacles[tracked_id]->cov * H.transpose() + Q;
            }

            Eigen::Vector2f z_actual(range, bearing);

            p.back().push_back((z_actual - z_pred).transpose() * Psi.inverse() *
                               (z_actual - z_pred));
        }
    }

    std::vector<int> match;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::greedy_data_association(m_tracked_obstacles, detected_obstacles, p, m_new_obj_slam_thres);

    // Update vectors, now with known correspondence
    //  RCLCPP_INFO(this->get_logger(), "UPDATE WITH KNOWN CORRESPONDENCE");
    for (size_t i = 0; i < detected_obstacles.size(); i++) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = all_seaing_perception::local_to_range_bearing_signature(
            detected_obstacles[i]->local_centroid, detected_obstacles[i]->label);
        Eigen::Vector2f z_actual(range, bearing);
        if (match[i] == -1) {
            // increase object count and expand & initialize matrices
            m_num_obj++;
            // add object to tracked obstacles vector
            detected_obstacles[i]->id = m_obstacle_id++;
            m_tracked_obstacles.push_back(detected_obstacles[i]);
            Eigen::Matrix<float, 2, 2> init_new_cov{
                {(float)m_init_new_cov, 0},
                {0, (float)m_init_new_cov},
            };
            if (m_track_robot) {
                m_state.conservativeResize(3 + 2 * m_num_obj);
                m_state.tail(2) = Eigen::Vector2f(m_state(0), m_state(1)) +
                                  range * Eigen::Vector2f(std::cos(bearing + m_state(2)),
                                                          std::sin(bearing + m_state(2)));
                m_cov.conservativeResizeLike(
                    Eigen::MatrixXf::Zero(3 + 2 * m_num_obj, 3 + 2 * m_num_obj));
                m_cov.bottomRightCorner(2, 2) = init_new_cov;
            } else {
                m_tracked_obstacles.back()->mean_pred =
                    Eigen::Vector2f(m_nav_x, m_nav_y) +
                    range * Eigen::Vector2f(std::cos(bearing + m_nav_heading),
                                            std::sin(bearing + m_nav_heading));
                m_tracked_obstacles.back()->cov = init_new_cov;
            }
        }
        int tracked_id = match[i] >= 0 ? match[i] : m_num_obj - 1;
        if (m_track_robot) {
            float d_x = m_state(3 + 2 * tracked_id) - m_state(0);
            float d_y = m_state(3 + 2 * tracked_id + 1) - m_state(1);
            float q = d_x * d_x + d_y * d_y;
            Eigen::Matrix<float, 2, 5> h{
                {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x,
                 std::sqrt(q) * d_y},
                {d_y, -d_x, -1, -d_y, d_x},
            };
            Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_state(2));
            Eigen::MatrixXf F = Eigen::MatrixXf::Zero(5, 3 + 2 * m_num_obj);
            F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
            F.block(3, 3 + 2 * tracked_id, 2, 2) = Eigen::Matrix2f::Identity();
            Eigen::MatrixXf H = h * F / q;
            Eigen::MatrixXf K = m_cov * H.transpose() * (H * m_cov * H.transpose() + Q).inverse();
            m_state += K * (z_actual - z_pred);
            m_cov =
                (Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) - K * H) * m_cov;
        } else {
            float d_x = m_tracked_obstacles[tracked_id]->mean_pred[0] - m_nav_x;
            float d_y = m_tracked_obstacles[tracked_id]->mean_pred[1] - m_nav_y;
            float q = d_x * d_x + d_y * d_y;
            Eigen::Matrix<float, 2, 2> h{
                {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                {-d_y, d_x},
            };
            Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
            Eigen::MatrixXf H = h / q;
            Eigen::MatrixXf K =
                m_tracked_obstacles[tracked_id]->cov * H.transpose() *
                (H * m_tracked_obstacles[tracked_id]->cov * H.transpose() + Q).inverse();
            m_tracked_obstacles[tracked_id]->mean_pred += K * (z_actual - z_pred);
            m_tracked_obstacles[tracked_id]->cov =
                (Eigen::Matrix2f::Identity() - K * H) * m_tracked_obstacles[tracked_id]->cov;
            detected_obstacles[i]->mean_pred = m_tracked_obstacles[tracked_id]->mean_pred;
            detected_obstacles[i]->cov = m_tracked_obstacles[tracked_id]->cov;
        }

        // update data for matched obstacles (we'll update position after we update SLAM with all
        // points)
        detected_obstacles[i]->id = m_tracked_obstacles[tracked_id]->id;
        m_tracked_obstacles[tracked_id] = detected_obstacles[i];
    }
    
    this->update_maps();

    pcl::PointXYZ p0(0, 0, 0);
    float avg_dist = 0;
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        avg_dist += pcl::euclideanDistance(p0, m_tracked_obstacles[i]->local_centroid) / ((float)m_tracked_obstacles.size());
    }

    // Filter old obstacles
    std::vector<int> to_remove;
    std::vector<int> to_keep;
    std::vector<int> to_keep_flat = {0, 1, 2};
    for (int tracked_id = 0; tracked_id < static_cast<int>(m_tracked_obstacles.size());
         tracked_id++) {
        if (chosen_tracked.count(tracked_id)) {
            to_keep.push_back(tracked_id);
            if (m_track_robot) {
                to_keep_flat.insert(to_keep_flat.end(),
                                    {3 + 2 * tracked_id, 3 + 2 * tracked_id + 1});
            }
            continue;
        }
        // Check if in FoV (which'll mean it's dead, at least temporarily)
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = m_tracked_obstacles[tracked_id]->local_centroid.x;
        lidar_point.y = m_tracked_obstacles[tracked_id]->local_centroid.y;
        lidar_point.z = m_tracked_obstacles[tracked_id]->local_centroid.z;
        geometry_msgs::msg::Point camera_point =
            lidar_point; // ALREADY IN THE SAME FRAME, WAS TRANSFORMED BEFORE BEING PUBLISHED BY
                         // bbox_project_pcloud.cpp
        cv::Point2d xy_rect =
            m_is_sim ? custom_project(m_cam_model,
                           cv::Point3d(camera_point.y, camera_point.z, -camera_point.x))
                     : custom_project(m_cam_model,
                           cv::Point3d(camera_point.x, camera_point.y, camera_point.z));
        ;
        // RCLCPP_INFO(this->get_logger(), "OBSTACLE ID %d (%lf, %lf, %lf)->(%lf, %lf)",
        // m_tracked_obstacles[tracked_id]->id, lidar_point.x, lidar_point.y, lidar_point.z,
        // xy_rect.x, xy_rect.y);
        if (((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
             (xy_rect.y < m_cam_model.cameraInfo().height) && (lidar_point.x >= 0)) ||
            !m_check_fov) {
            // Dead
            if (m_tracked_obstacles[tracked_id]->is_dead) {
                // Was also dead before, add time dead
                // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d TIME PERIOD FROM PREVIOUS DEAD: %lf
                // - %lf", tracked_id, m_tracked_obstacles[tracked_id]->last_dead.seconds(),
                // rclcpp::Time(m_local_header.stamp).seconds());
                m_tracked_obstacles[tracked_id]->time_dead =
                    rclcpp::Time(m_local_header.stamp) -
                    m_tracked_obstacles[tracked_id]->last_dead +
                    m_tracked_obstacles[tracked_id]->time_dead;
                // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d DEAD FOR %lf SECONDS, OBSTACLE DROP
                // THRESHOLD: %lf", tracked_id,
                // m_tracked_obstacles[tracked_id]->time_dead.seconds(), m_obstacle_drop_thresh);
                if (m_tracked_obstacles[tracked_id]->time_dead.seconds() >
                    (m_normalize_drop_thresh ? (m_obstacle_drop_thresh *
                        (pcl::euclideanDistance(p0,
                                                m_tracked_obstacles[tracked_id]->local_centroid) /
                         avg_dist) *
                        m_normalize_drop_dist) : m_obstacle_drop_thresh)) {
                    // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d/%d DROPPED", tracked_id,
                    // m_num_obj);
                    to_remove.push_back(tracked_id);
                    continue;
                }
            }
            m_tracked_obstacles[tracked_id]->is_dead = true;
            m_tracked_obstacles[tracked_id]->last_dead = m_local_header.stamp;
        } else {
            m_tracked_obstacles[tracked_id]->is_dead = false;
        }
        to_keep.push_back(tracked_id);
        if (m_track_robot) {
            to_keep_flat.insert(to_keep_flat.end(), {3 + 2 * tracked_id, 3 + 2 * tracked_id + 1});
        }
    }

    // update vectors & matrices
    if (!to_remove.empty()) {
        std::vector<std::shared_ptr<all_seaing_perception::ObjectCloud>> new_obj;

        for (int i : to_keep) {
            new_obj.push_back(m_tracked_obstacles[i]);
        }

        if (m_track_robot) {
            Eigen::MatrixXf new_state(3 + 2 * to_keep.size(), 1);
            Eigen::MatrixXf new_cov(3 + 2 * to_keep.size(), 3 + 2 * to_keep.size());
            int new_i = 0;
            for (int i : to_keep_flat) {
                new_state(new_i) = m_state(i);
                int new_j = 0;
                for (int j : to_keep_flat) {
                    new_cov(new_i, new_j) = m_cov(i, j);
                    new_j++;
                }
                new_i++;
            }
            m_state = new_state;
            m_cov = new_cov;
        }

        m_tracked_obstacles = new_obj;
        m_num_obj = to_keep.size();
    }

    if (m_track_robot) {
        m_trace.push_back(std::make_pair(m_state(0), m_state(1)));
    }

    this->publish_maps();

    // RCLCPP_INFO(this->get_logger(), "AFTER TRACKED MAP PUBLISHING");
    if(m_got_nav){
        this->visualize_predictions();
    }
    if(m_track_robot && m_got_nav && m_got_odom){
        publish_slam();
    }
}

ObjectTrackingMap::~ObjectTrackingMap() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMap>());
    rclcpp::shutdown();
    return 0;
}
