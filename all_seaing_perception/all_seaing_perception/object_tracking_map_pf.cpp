#include "all_seaing_perception/object_tracking_map_pf.hpp"

ObjectTrackingMapPF::ObjectTrackingMapPF() : Node("object_tracking_map_pf") {
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<std::string>("slam_frame_id", "slam_map");
    this->declare_parameter<int>("num_particles", 100);
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
    this->declare_parameter<double>("init_new_cov", 10.0);
    this->declare_parameter<bool>("imu_predict", true);
    this->declare_parameter<bool>("gps_update", true);
    this->declare_parameter<double>("normalize_drop_dist", 1.0);
    this->declare_parameter<double>("odom_refresh_rate", 40);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_slam_frame_id = this->get_parameter("slam_frame_id").as_string();
    m_num_particles = this->get_parameter("num_particles").as_int();
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
    m_init_new_cov = this->get_parameter("init_new_cov").as_double();
    m_imu_predict = this->get_parameter("imu_predict").as_bool();
    m_gps_update = this->get_parameter("gps_update").as_bool();
    
    m_normalize_drop_dist = this->get_parameter("normalize_drop_dist").as_double();
    m_odom_refresh_rate = this->get_parameter("odom_refresh_rate").as_double();

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    this->declare_parameter<bool>("check_fov", false);
    m_check_fov = this->get_parameter("check_fov").as_bool();

    this->declare_parameter<bool>("direct_tf", false);
    m_direct_tf = this->get_parameter("direct_tf").as_bool();

    this->declare_parameter<bool>("normalize_drop_thresh", false);
    m_normalize_drop_thresh = this->get_parameter("normalize_drop_thresh").as_bool();

    this->declare_parameter<bool>("include_odom_theta", false);
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
            std::bind(&ObjectTrackingMapPF::object_track_map_publish, this, std::placeholders::_1));
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_topic", 10,
        std::bind(&ObjectTrackingMapPF::intrinsics_cb, this, std::placeholders::_1));

    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    // update odometry based on transforms on timer click
    odom_timer = this->create_wall_timer(
    std::chrono::duration<float>(((float)1.0)/m_odom_refresh_rate), std::bind(&ObjectTrackingMapPF::odom_callback, this));

    // publish computed global robot position as a transform slam_map->map and a message with the pose of the robot wrt slam_map
    m_slam_pub = this->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/tracked", 10);
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // update odometry based on IMU data by subscribing to the odometry message
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&ObjectTrackingMapPF::odom_msg_callback, this, std::placeholders::_1));

    m_first_state = true;
    m_got_local_frame = false;
    m_got_nav = false;
    m_got_odom = false;

    m_best_particle_index = -1;
}

void ObjectTrackingMapPF::publish_slam(){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = m_last_odom_msg.header.stamp;
    // t.header.stamp = this->get_clock()->now();
    if(m_direct_tf){
        // publish the transform from slam_map to the local frame (camera/lidar)
        t.header.frame_id = m_slam_frame_id;
        t.child_frame_id = m_local_frame_id;
        t.transform.translation.x = m_state(0);
        t.transform.translation.y = m_state(1);
        t.transform.translation.z = m_nav_z;

        tf2::Quaternion q;
        q.setRPY(0, 0, m_state(2));
        t.transform.rotation = tf2::toMsg(q);
    }else{
        // transform from slam_map to map, s.t. slam_map->robot is predicted pose
        t.header.frame_id = m_slam_frame_id;
        t.child_frame_id = m_global_frame_id;
        double shift_x, shift_y, shift_theta;
        // (slam_map->robot)@(robot->map) = (slam_map->robot)@inv(map->robot)
        std::tie(shift_x, shift_y, shift_theta) = all_seaing_perception::ObjectTrackingMap::compose_transforms(std::make_tuple(m_state(0), m_state(1), m_state(2)), all_seaing_perception::ObjectTrackingMap::compute_transform_from_to(m_nav_x, m_nav_y, m_nav_heading, 0, 0, 0));
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

void ObjectTrackingMapPF::odom_msg_callback(const nav_msgs::msg::Odometry &msg){
    // TODO: ODOM MESSAGE (IMU LINEAR + ANGULAR) CALLBACK
    // !!! THOSE ARE RELATIVE TO THE ROBOT AND DEPENDENT ON ITS HEADING, USE THE CORRECT MOTION MODEL
    if(m_is_sim){
        m_nav_vx = msg.twist.twist.linear.x;
        m_nav_vy = msg.twist.twist.linear.y;   
    }else{
        // Pixhawk rotated to the left, facing up, so need to rotate the acceleration vector accordingly
        m_nav_vx = -msg.twist.twist.linear.y;
        m_nav_vy = msg.twist.twist.linear.x;
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

    if (!m_got_nav || !m_imu_predict || m_first_state) return;

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
    
    // to see the robot position prediction mean & uncertainty
    if(m_got_nav){
        this->visualize_predictions();
    }
    // publish robot pose predictions (here and after measurement update) as transforms from map to lidar/camera frame, possibly also as a message
    // it should ultimately be a source of odometry that can be used by other nodes along with the global map
    if(m_got_nav && m_got_odom){
        publish_slam();
    }
}

void ObjectTrackingMapPF::odom_callback() {
    // TODO: ODOM TRANSFORM (GPS + HEADING) CALLBACK
    if(!m_got_local_frame) return;

    //update odometry transforms
    //TODO: add a flag for each one that says if they succedeed, to know to continue or not
    // RCLCPP_INFO(this->get_logger(), "ODOM CALLBACK");
    m_map_lidar_tf = get_tf(m_global_frame_id, m_local_frame_id);
    m_lidar_map_tf = get_tf(m_local_frame_id, m_global_frame_id);

    m_nav_z = m_map_lidar_tf.transform.translation.z;
    tf2::Quaternion quat;
    tf2::fromMsg(m_map_lidar_tf.transform.rotation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    m_nav_heading = y;

    m_got_nav = true;

    // to ignore gps TFs that are the same thing many times a second due to lag and cause SLAM to lock into the GPS
    if((!m_first_state) && (m_nav_x == m_map_lidar_tf.transform.translation.x) && (m_nav_y == m_map_lidar_tf.transform.translation.y)) return;

    m_nav_x = m_map_lidar_tf.transform.translation.x;
    m_nav_y = m_map_lidar_tf.transform.translation.y;


    if (m_first_state) {
        // initialize mean and cov robot pose
        m_state = Eigen::Vector3f(m_nav_x, m_nav_y, m_nav_heading);
        Eigen::Matrix3f init_pose_noise{
            {m_gps_xy_noise, 0, 0},
            {0, m_gps_xy_noise, 0},
            {0, 0, m_gps_theta_noise},
        };
        m_cov = init_pose_noise;
        // m_cov = Eigen::Matrix3f::Zero();
        m_first_state = false;
        // m_last_odom_time = rclcpp::Time(msg.header.stamp);
        return;
    }

    if (!m_imu_predict) {

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

    if(m_got_nav){
        this->visualize_predictions();
    }
    if(m_got_nav && m_got_odom){
        publish_slam();
    }
}

void ObjectTrackingMapPF::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    // RCLCPP_INFO(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

SLAMParticle::SLAMParticle(float init_x, float init_y, float init_theta){
    m_pose = Eigen::Vector3f(init_x, init_y, init_theta);
    m_num_obj = 0;
    m_obstacle_id = 0;
    m_got_gps = false;
}

void SLAMParticle::sample_pose(double dx, double dy, double dtheta, float dt, float xy_noise, float theta_noise){
    // TODO: SAMPLE NEW POSE GIVEN PARTICLE POSE AND ODOMETRY DATA AND UNCERTAINTIES
}

void SLAMParticle::update_gps(double x, double y, double theta, float xy_uncertainty, float theta_uncertainty){
    // TODO: GPS UPDATE (SAME EKF UPDATE BUT JUST ON THE 3X3 GPS MATRIX THAT FUSES ALL THE MEASUREMENTS RECEIVED)
}

void SLAMParticle::update_map(std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles, builtin_interfaces::msg::Time curr_time,
    bool is_sim, float range_std, float bearing_std, float init_new_cov, float new_obj_slam_thres,
    bool check_fov, float obstacle_drop_thres, bool normalize_drop_thres, image_geometry::PinholeCameraModel cam_model){
    
    Eigen::Matrix<float, 2, 2> Q{
        {range_std, 0},
        {0, bearing_std},
    };

    std::vector<std::vector<float>> p;
    for (std::shared_ptr<ObjectCloud> det_obs : detected_obstacles) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) =
            all_seaing_perception::ObjectTrackingMap::local_to_range_bearing_signature(det_obs->local_centroid, det_obs->label);
        p.push_back(std::vector<float>());
        Eigen::Vector2f z_pred;
        Eigen::MatrixXf Psi;
        for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
            float d_x = m_tracked_obstacles[tracked_id]->mean_pred[0] - m_pose(0);
            float d_y = m_tracked_obstacles[tracked_id]->mean_pred[1] - m_pose(1);
            float q = d_x * d_x + d_y * d_y;
            z_pred = Eigen::Vector2f(std::sqrt(q), std::atan2(d_y, d_x) - m_pose(2));

            Eigen::Matrix<float, 2, 2> h{
                {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
                {-d_y, d_x},
            };

            Eigen::MatrixXf H = h / q;
            Psi = H * m_tracked_obstacles[tracked_id]->cov * H.transpose() + Q;

            Eigen::Vector2f z_actual(range, bearing);

            p.back().push_back((z_actual - z_pred).transpose() * Psi.inverse() *
                               (z_actual - z_pred));
        }
    }

    vector<int> match;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    std::tie(match, chosen_detected, chosen_tracked) = all_seaing_perception::ObjectTrackingMap::greedy_data_association(m_tracked_obstacles, detected_obstacles, p, new_obj_slam_thres);

    // Update vectors, now with known correspondence
    for (size_t i = 0; i < detected_obstacles.size(); i++) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = all_seaing_perception::ObjectTrackingMap::local_to_range_bearing_signature(
            detected_obstacles[i]->local_centroid, detected_obstacles[i]->label);
        Eigen::Vector2f z_actual(range, bearing);
        if (match[i] == -1) {
            // increase object count and expand & initialize matrices
            m_num_obj++;
            // add object to tracked obstacles vector
            detected_obstacles[i]->id = m_obstacle_id++;
            m_tracked_obstacles.push_back(detected_obstacles[i]);
            Eigen::Matrix<float, 2, 2> matrix_init_new_cov{
                {(float)init_new_cov, 0},
                {0, (float)init_new_cov},
            };
            m_tracked_obstacles.back()->mean_pred =
                Eigen::Vector2f(pose(0), m_pose(1)) +
                range * Eigen::Vector2f(std::cos(bearing + m_pose(2)),
                                        std::sin(bearing + m_pose(2)));
            m_tracked_obstacles.back()->cov = matrix_init_new_cov;
        }
        int tracked_id = match[i] > 0 ? match[i] : m_num_obj - 1;
        float d_x = m_tracked_obstacles[tracked_id]->mean_pred[0] - m_pose(0);
        float d_y = m_tracked_obstacles[tracked_id]->mean_pred[1] - m_pose(1);
        float q = d_x * d_x + d_y * d_y;
        Eigen::Matrix<float, 2, 2> h{
            {std::sqrt(q) * d_x, std::sqrt(q) * d_y},
            {-d_y, d_x},
        };
        Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_pose(2));
        Eigen::MatrixXf H = h / q;
        Eigen::MatrixXf K =
            m_tracked_obstacles[tracked_id]->cov * H.transpose() *
            (H * m_tracked_obstacles[tracked_id]->cov * H.transpose() + Q).inverse();
        m_tracked_obstacles[tracked_id]->mean_pred += K * (z_actual - z_pred);
        m_tracked_obstacles[tracked_id]->cov =
            (Eigen::Matrix2f::Identity() - K * H) * m_tracked_obstacles[tracked_id]->cov;
        detected_obstacles[i]->mean_pred = m_tracked_obstacles[tracked_id]->mean_pred;
        detected_obstacles[i]->cov = m_tracked_obstacles[tracked_id]->cov;

        // update data for matched obstacles (we'll update position after we update SLAM with all points)
        detected_obstacles[i]->id = m_tracked_obstacles[tracked_id]->id;
        m_tracked_obstacles[tracked_id] = detected_obstacles[i];
    }

    // Update all (new and old, since they also have correlation with each other) objects global
    // positions (including the cloud, and then the local points respectively)
    pcl::PointXYZ p0(0, 0, 0);
    float avg_dist = 0;
    for (size_t i = 0; i < m_tracked_obstacles.size(); i++) {
        pcl::PointXYZ upd_glob_centr = m_tracked_obstacles[i]->global_centroid;
        upd_glob_centr.x = m_tracked_obstacles[i]->mean_pred[0];
        upd_glob_centr.y = m_tracked_obstacles[i]->mean_pred[1];
        pcl::PointXYZ upd_loc_centr;
        upd_loc_centr = all_seaing_perception::ObjectTrackingMap::convert_to_local(upd_glob_centr);

        // move everything based on the difference between that and the previous assumed global (then local) centroid
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        for (pcl::PointXYZHSV &global_pt : m_tracked_obstacles[i]->global_pcloud_ptr->points) {
            global_pt.x += upd_glob_centr.x - m_tracked_obstacles[i]->global_centroid.x;
            global_pt.y += upd_glob_centr.y - m_tracked_obstacles[i]->global_centroid.y;
            upd_local_obj_pcloud->push_back(
                all_seaing_perception::ObjectTrackingMap::convert_to_local(global_pt));
        }

        m_tracked_obstacles[i]->global_centroid = upd_glob_centr;
        m_tracked_obstacles[i]->local_centroid = upd_loc_centr;
        avg_dist += pcl::euclideanDistance(p0, upd_loc_centr) / ((float)m_tracked_obstacles.size());
    }

    // Filter old obstacles
    std::vector<int> to_remove;
    std::vector<int> to_keep;
    for (int tracked_id = 0; tracked_id < static_cast<int>(m_tracked_obstacles.size());
         tracked_id++) {
        if (chosen_tracked.count(tracked_id)) {
            to_keep.push_back(tracked_id);
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
            is_sim ? cam_model.project3dToPixel(
                           cv::Point3d(camera_point.y, camera_point.z, -camera_point.x))
                     : cam_model.project3dToPixel(
                           cv::Point3d(camera_point.x, camera_point.y, camera_point.z));
        ;

        if (((xy_rect.x >= 0) && (xy_rect.x < cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
             (xy_rect.y < cam_model.cameraInfo().height) && (lidar_point.x >= 0)) ||
            !check_fov) {
            // Dead
            if (m_tracked_obstacles[tracked_id]->is_dead) {
                // Was also dead before, add time dead
                m_tracked_obstacles[tracked_id]->time_dead =
                    rclcpp::Time(curr_time) -
                    m_tracked_obstacles[tracked_id]->last_dead +
                    m_tracked_obstacles[tracked_id]->time_dead;

                if (m_tracked_obstacles[tracked_id]->time_dead.seconds() >
                    (normalize_drop_thresh ? (obstacle_drop_thresh *
                        (pcl::euclideanDistance(p0,
                                                m_tracked_obstacles[tracked_id]->local_centroid) /
                         avg_dist) *
                        normalize_drop_dist) : obstacle_drop_thresh)) {
                    // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d/%d DROPPED", tracked_id, m_num_obj);
                    to_remove.push_back(tracked_id);
                    continue;
                }
            }
            m_tracked_obstacles[tracked_id]->is_dead = true;
            m_tracked_obstacles[tracked_id]->last_dead = curr_time;
        } else {
            m_tracked_obstacles[tracked_id]->is_dead = false;
        }
    }

    // update vectors & matrices
    if (!to_remove.empty()) {
        std::vector<std::shared_ptr<ObjectCloud>> new_obj;

        for (int i : to_keep) {
            new_obj.push_back(m_tracked_obstacles[i]);
        }

        m_tracked_obstacles = new_obj;
        m_num_obj = to_keep.size();
    }
}

void ObjectTrackingMapPF::visualize_predictions(){
    // delete previous markers
    visualization_msgs::msg::MarkerArray delete_arr;
    visualization_msgs::msg::Marker delete_mark;
    delete_mark.id = 0;
    delete_mark.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_arr.markers.push_back(delete_mark);
    m_map_cov_viz_pub->publish(delete_arr);

    visualization_msgs::msg::MarkerArray ellipse_arr;
    marker_id = 0;
    for (int i = 0; i < m_num_particles; i++){
        // show particle itself
        ellipse_arr.markers.extend(m_particles[i]->visualize_pose(m_global_header, m_slam_header_id, marker_id).markers);
        if (i == m_best_particle_index){
            // show particle and map
            ellipse_arr.markers.extend(m_particles[i]->visualize_map(m_global_header, m_slam_header_id, marker_id).markers);
        }
    }
    m_map_cov_viz_pub->publish(ellipse_arr)
}

visualization_msgs::msg::MarkerArray SLAMParticle::visualize_pose(std_msgs::msg::Header global_header, string slam_frame_id, int &id) {
    visualization_msgs::msg::MarkerArray ellipse_arr;

    visualization_msgs::msg::Marker ellipse;
    ellipse.type = visualization_msgs::msg::Marker::SPHERE;
    ellipse.pose.position.x = m_pose(0);
    ellipse.pose.position.y = m_pose(1);
    ellipse.pose.position.z = 0;
    ellipse.pose.orientation = tf2::toMsg(quat_rot);

    ellipse.scale.x = 0.5;
    ellipse.scale.y = 0.5;
    ellipse.scale.z = 0.5;
    ellipse.color.a = 1;
    ellipse.color.g = 1;
    ellipse.header = global_header;
    ellipse.header.frame_id = slam_frame_id;
    ellipse.id = id_start++;
    ellipse_arr.markers.push_back(ellipse);

    visualization_msgs::msg::Marker angle_marker;
    angle_marker.type = visualization_msgs::msg::Marker::ARROW;
    angle_marker.pose.position.x = m_pose(0);
    angle_marker.pose.position.y = m_pose(1);
    angle_marker.pose.position.z = 0;
    tf2::Quaternion angle_quat;
    angle_quat.setRPY(0, 0, m_state(2));
    angle_marker.pose.orientation = tf2::toMsg(angle_quat);

    angle_marker.scale.x = 0.5;
    angle_marker.scale.y = 0.2;
    angle_marker.scale.z = 0.2;
    angle_marker.color.a = 1;
    angle_marker.header = global_header;
    angle_marker.header.frame_id = slam_frame_id;
    angle_marker.id = id_start++;
    ellipse_arr.markers.push_back(angle_marker);

    return ellipse_arr;
}

visualization_msgs::msg::MarkerArray SLAMParticle::visualize_map(std_msgs::msg::Header local_header, string slam_frame_id, float new_obj_slam_thres, int &id_start) {
    visualization_msgs::msg::MarkerArray ellipse_arr;

    // obstacle predictions
    for (int i = 0; i < m_num_obj; i++) {
        Eigen::Vector2f obj_mean = m_tracked_obstacles[i]->mean_pred;
        Eigen::Matrix2f obj_cov = m_tracked_obstacles[i]->cov;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(obj_cov);
        double a_x = eigen_solver.eigenvalues()(0);
        double a_y = eigen_solver.eigenvalues()(1);

        Eigen::Vector2f axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2f axis_y = eigen_solver.eigenvectors().col(1);
        double cov_scale = new_obj_slam_thres; // to visualize the threshold where new obstacles are added
        tf2::Matrix3x3 rot_mat(axis_x(0), axis_y(0), 0, axis_x(1), axis_y(1), 0, 0, 0, 1);
        tf2::Quaternion quat_rot;
        rot_mat.getRotation(quat_rot);
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;

        ellipse.pose.position.x = obj_mean(0);
        ellipse.pose.position.y = obj_mean(1);
        ellipse.pose.orientation = tf2::toMsg(quat_rot);

        ellipse.pose.position.z = 0;
        ellipse.scale.x = cov_scale * sqrt(a_x);
        ellipse.scale.y = cov_scale * sqrt(a_y);
        ellipse.scale.z = 1;
        ellipse.color.a = 1;
        ellipse.color.r = 1;
        ellipse.header = global_header;
        ellipse.header.frame_id = slam_frame_id;
        ellipse.id = id_start++;
        ellipse_arr.markers.push_back(ellipse);
    }

    return ellipse_arr;
}

void ObjectTrackingMapPF::object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg){
    if(msg->objects.size() == 0) return;    

    // Set up headers and transforms
    m_local_header = msg->objects[0].cloud.header;
    m_global_header.frame_id = m_slam_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    m_local_frame_id = m_local_header.frame_id;
    m_got_local_frame = true;

    m_lidar_map_tf = get_tf(m_global_frame_id, m_local_frame_id);
    m_map_lidar_tf = get_tf(m_local_frame_id, m_global_frame_id);

    if(m_first_state) return;

    std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles;
    for (all_seaing_interfaces::msg::LabeledObjectPointCloud obj : msg->objects) {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(obj.cloud, *local_obj_pcloud);
        for (pcl::PointXYZHSV &pt : local_obj_pcloud->points) {
            pcl::PointXYZHSV global_pt =
                all_seaing_perception::ObjectTrackingMap::convert_to_global(pt);
            global_obj_pcloud->push_back(global_pt);
        }
        std::shared_ptr<ObjectCloud> obj_cloud(
            new ObjectCloud(obj.time, obj.label, local_obj_pcloud, global_obj_pcloud));
        detected_obstacles.push_back(obj_cloud);
    }

    // Make and publish untracked map
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> untracked_obs;
    std::vector<int> untracked_labels;
    for (std::shared_ptr<ObjectCloud> det_obs : detected_obstacles) {
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
            new all_seaing_perception::Obstacle(m_local_header, m_global_header, raw_cloud, ind,
                                                m_obstacle_id++, m_lidar_map_tf));
        untracked_labels.push_back(det_obs->label);
        untracked_obs.push_back(untracked_ob);
    }
    all_seaing_perception::ObjectTrackingMap::publish_map(m_local_header, m_global_header, "untracked", true, untracked_obs, m_untracked_map_pub,
                      untracked_labels);

    // TODO: FastSLAM ("Probabilistic Robotics", Seb. Thrun, inspired implementation)

    // Publish map with tracked obstacles
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> tracked_obs;
    std::vector<int> tracked_labels;
    for (std::shared_ptr<ObjectCloud> t_ob : m_tracked_obstacles) {
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
    all_seaing_perception::ObjectTrackingMap::publish_map(m_local_header, m_global_header, "tracked", true, tracked_obs, m_tracked_map_pub,
                      tracked_labels);
    
    if(m_got_nav){
        this->visualize_predictions();
    }
    if( m_got_nav && m_got_odom){
        publish_slam();
    }
}

ObjectTrackingMapPF::~ObjectTrackingMapPF() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMapPF>());
    rclcpp::shutdown();
    return 0;
}
