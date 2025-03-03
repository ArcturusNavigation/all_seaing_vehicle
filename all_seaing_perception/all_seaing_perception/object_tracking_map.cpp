#include "all_seaing_perception/object_tracking_map.hpp"

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map") {
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("range_uncertainty", 1.0);
    this->declare_parameter<double>("bearing_uncertainty", 1.0);
    this->declare_parameter<double>("motion_xy_noise", 1.0);
    this->declare_parameter<double>("motion_theta_noise", 1.0);
    this->declare_parameter<double>("new_object_slam_threshold", 1.0);
    this->declare_parameter<double>("init_new_cov", 10.0);
    this->declare_parameter<bool>("track_robot", false);
    this->declare_parameter<double>("normalize_drop_dist", 1.0);
    this->declare_parameter<double>("odom_refresh_rate", 1000);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_range_std = this->get_parameter("range_uncertainty").as_double();
    m_bearing_std = this->get_parameter("bearing_uncertainty").as_double();
    m_xy_noise = this->get_parameter("motion_xy_noise").as_double();
    m_theta_noise = this->get_parameter("motion_theta_noise").as_double();
    m_new_obj_slam_thres = this->get_parameter("new_object_slam_threshold").as_double();
    m_init_new_cov = this->get_parameter("init_new_cov").as_double();
    m_track_robot = this->get_parameter("track_robot").as_bool();
    m_normalize_drop_dist = this->get_parameter("normalize_drop_dist").as_double();
    m_odom_refresh_rate = this->get_parameter("odom_refresh_rate").as_double();

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    this->declare_parameter<bool>("check_fov", false);
    m_check_fov = this->get_parameter("check_fov").as_bool();

    // Initialize navigation variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
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

    odom_timer = this->create_wall_timer(
      std::chrono::duration<float>(((float)1.0)/m_odom_refresh_rate), std::bind(&ObjectTrackingMap::odom_callback, this));

    m_first_state = true;
    m_got_local_frame = false;
}

ObjectCloud::ObjectCloud(rclcpp::Time t, int l, pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc,
                         pcl::PointCloud<pcl::PointXYZHSV>::Ptr glob) {
    id = -1;
    label = l;
    time_seen = t;
    last_dead = rclcpp::Time(0);
    time_dead = rclcpp::Duration(0, 0);
    is_dead = false;
    global_pcloud_ptr = glob;
    for (pcl::PointXYZHSV &global_pt : global_pcloud_ptr->points) {
        global_centroid.x += global_pt.x / ((double)global_pcloud_ptr->points.size());
        global_centroid.y += global_pt.y / ((double)global_pcloud_ptr->points.size());
        global_centroid.z += global_pt.z / ((double)global_pcloud_ptr->points.size());
    }
    this->update_loc_pcloud(loc);
}

void ObjectCloud::update_loc_pcloud(pcl::PointCloud<pcl::PointXYZHSV>::Ptr loc) {
    local_pcloud_ptr = loc;
    for (pcl::PointXYZHSV &local_pt : local_pcloud_ptr->points) {
        local_centroid.x += local_pt.x / ((double)local_pcloud_ptr->points.size());
        local_centroid.y += local_pt.y / ((double)local_pcloud_ptr->points.size());
        local_centroid.z += local_pt.z / ((double)local_pcloud_ptr->points.size());
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

void ObjectTrackingMap::odom_callback() {
    if(!m_got_local_frame) return;

    //update odometry transforms
    //TODO: add a flag for each one that says if they succedeed, to know to continue or not
    // RCLCPP_INFO(this->get_logger(), "ODOM CALLBACK");
    m_lidar_map_tf = get_tf(m_global_frame_id, m_local_frame_id);
    m_map_lidar_tf = get_tf(m_local_frame_id, m_global_frame_id);

    // RCLCPP_INFO(this->get_logger(), "GOT ODOM");
    m_nav_x = m_map_lidar_tf.transform.translation.x;
    m_nav_y = m_map_lidar_tf.transform.translation.y;
    m_nav_z = m_map_lidar_tf.transform.translation.z;

    tf2::Quaternion quat;
    tf2::fromMsg(m_map_lidar_tf.transform.rotation, quat);
    tf2::Matrix3x3 m(quat);
    double r, p, y;
    m.getRPY(r, p, y);
    m_nav_heading = y;

    if (m_track_robot) {
        if (m_first_state) {
            // initialize mean and cov robot pose
            m_state = Eigen::Vector3f(m_nav_x, m_nav_y, m_nav_heading);
            Eigen::Matrix3f init_pose_noise{
                {m_xy_noise, 0, 0},
                {0, m_xy_noise, 0},
                {0, 0, m_theta_noise},
            };
            m_cov = init_pose_noise;
            // m_cov = Eigen::Matrix3f::Zero();
            m_first_state = false;
            // m_last_odom_time = rclcpp::Time(msg.header.stamp);
            return;
        }

        // rclcpp::Time curr_odom_time = rclcpp::Time(msg.header.stamp);
        // float dt = (curr_odom_time - m_last_odom_time).seconds();
        // m_last_odom_time = curr_odom_time;
        // m_nav_omega = (m_nav_heading - m_state[2]) / dt;

        // RCLCPP_INFO(this->get_logger(), "ROBOT ODOM DATA: (%lf, %lf, %lf), pub omega: %lf, dt:
        // %lf, comp omega: %lf", m_nav_x, m_nav_y, m_nav_heading, msg.twist.twist.angular.z, dt,
        // (m_nav_heading-m_state[2])/dt);

        Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3, 3 + 2 * m_num_obj);
        F.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();

        Eigen::Vector3f mot_const;
        Eigen::Matrix3f mot_grad = Eigen::Matrix3f::Zero();

        /*
        MOTION UPDATE MODEL:
        (x_old, y_old, theta_old) -> (x_old+v_comp_x*dt, y_old+v_comp_y*dt, theta_old+omega_comp*dt)
        -->gradient is identity matrix (that way we keep the correlations with the objects, if we
        set it to the new values it would be zero and delete them) remember that mot_grad is the
        gradients minus the identity matrix, so with this model it is the zero matrix
        */

        mot_const =
            Eigen::Vector3f(m_nav_x - m_state[0], m_nav_y - m_state[1], m_nav_heading - m_state[2]);
        // mot_grad is still zero

        m_state += F.transpose() * mot_const;
        Eigen::MatrixXf G = Eigen::MatrixXf::Identity(3 + 2 * m_num_obj, 3 + 2 * m_num_obj) +
                            F.transpose() * mot_grad * F;
        // add a consistent amount of noise based on how much the robot moved since the last time
        Eigen::Matrix3f motion_noise{
            {m_xy_noise * abs(mot_const[0]), 0, 0},
            {0, m_xy_noise * abs(mot_const[1]), 0},
            {0, 0, m_theta_noise * abs(mot_const[2])},
        };
        m_cov = G * m_cov * G.transpose() + F.transpose() * motion_noise * F;
        // to see the robot position prediction mean & uncertainty
        this->visualize_predictions();
    }
}

void ObjectTrackingMap::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    // RCLCPP_INFO(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

geometry_msgs::msg::TransformStamped ObjectTrackingMap::get_tf(const std::string &in_target_frame,
                                                               const std::string &in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "%s", ex.what());
    }
    return tf;
}

template <typename T>
T ObjectTrackingMap::convert_to_global(T point) {
    T new_point = point; // to keep color-related data
    geometry_msgs::msg::Point lc_pt_msg;
    lc_pt_msg.x = point.x;
    lc_pt_msg.y = point.y;
    lc_pt_msg.z = point.z;
    geometry_msgs::msg::Point gb_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(lc_pt_msg, gb_pt_msg, m_lidar_map_tf);
    new_point.x = gb_pt_msg.x;
    new_point.y = gb_pt_msg.y;
    new_point.z = gb_pt_msg.z;
    return new_point;
}

template <typename T>
T ObjectTrackingMap::convert_to_local(T point) {
    T new_point = point; // to keep color-related data
    geometry_msgs::msg::Point gb_pt_msg;
    gb_pt_msg.x = point.x;
    gb_pt_msg.y = point.y;
    gb_pt_msg.z = point.z;
    geometry_msgs::msg::Point lc_pt_msg;
    tf2::doTransform<geometry_msgs::msg::Point>(gb_pt_msg, lc_pt_msg, m_map_lidar_tf);
    new_point.x = lc_pt_msg.x;
    new_point.y = lc_pt_msg.y;
    new_point.z = lc_pt_msg.z;
    return new_point;
}

//(range, bearing, signature)
template <typename T>
ObjectTrackingMap::det_rbs ObjectTrackingMap::local_to_range_bearing_signature(T point, int label) {
    double range = std::hypot(point.x, point.y);
    double bearing = std::atan2(point.y, point.x);
    // RCLCPP_INFO(this->get_logger(), "LOCAL: (%lf, %lf) -> RBS: (%lf, %lf, %d)", point.x, point.y,
    // range, bearing, label);
    return det_rbs(range, bearing, label);
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
        ellipse.scale.x = sqrt(a_x);
        ellipse.scale.y = sqrt(a_y);
        ellipse.scale.z = 0.5;
        ellipse.color.a = 1;
        ellipse.color.g = 1;
        ellipse.header.frame_id = m_global_frame_id;
        ellipse.id = m_num_obj + 1;
        ellipse_arr.markers.push_back(ellipse);

        visualization_msgs::msg::Marker angle_marker;
        angle_marker.type = visualization_msgs::msg::Marker::ARROW;
        angle_marker.pose.position.x = robot_mean(0);
        angle_marker.pose.position.y = robot_mean(1);
        angle_marker.pose.position.z = 0;
        tf2::Quaternion angle_quat;
        angle_quat.setRPY(0, 0, m_state[2]);
        angle_marker.pose.orientation = tf2::toMsg(angle_quat);
        angle_marker.scale.x = m_cov(2, 2);
        angle_marker.scale.y = 0.2;
        angle_marker.scale.z = 0.2;
        angle_marker.color.a = 1;
        angle_marker.header.frame_id = m_global_frame_id;
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
        // RCLCPP_INFO(this->get_logger(), "OBJECT %d COVARIANCE AXES LENGTHS: (%lf, %lf)", i, a_x,
        // a_y);
        Eigen::Vector2f axis_x = eigen_solver.eigenvectors().col(0);
        Eigen::Vector2f axis_y = eigen_solver.eigenvectors().col(1);
        double cov_scale =
            m_new_obj_slam_thres; // to visualize the threshold where new obstacles are added
        tf2::Matrix3x3 rot_mat(axis_x(0), axis_y(0), 0, axis_x(1), axis_y(1), 0, 0, 0, 1);
        tf2::Quaternion quat_rot;
        rot_mat.getRotation(quat_rot);
        visualization_msgs::msg::Marker ellipse;
        ellipse.type = visualization_msgs::msg::Marker::SPHERE;
        ellipse.pose.position.x = obj_mean(0);
        ellipse.pose.position.y = obj_mean(1);
        ellipse.pose.position.z = 0;
        ellipse.pose.orientation = tf2::toMsg(quat_rot);
        ellipse.scale.x = cov_scale * sqrt(a_x);
        ellipse.scale.y = cov_scale * sqrt(a_y);
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
    // RCLCPP_INFO(this->get_logger(), "GOT DATA");

    // Set up headers and transforms
    m_local_header = msg->objects[0].cloud.header;
    m_global_header.frame_id = m_global_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    m_local_frame_id = m_local_header.frame_id;
    m_got_local_frame = true;

    // RCLCPP_INFO(this->get_logger(), "BEFORE GETTING ODOMETRY TF");
    m_lidar_map_tf = get_tf(m_global_frame_id, m_local_frame_id);
    m_map_lidar_tf = get_tf(m_local_frame_id, m_global_frame_id);
    // RCLCPP_INFO(this->get_logger(), "GOT ODOMETRY TF");

    if(m_track_robot && m_first_state) return;

    std::vector<std::shared_ptr<ObjectCloud>> detected_obstacles;
    for (all_seaing_interfaces::msg::LabeledObjectPointCloud obj : msg->objects) {
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(obj.cloud, *local_obj_pcloud);
        for (pcl::PointXYZHSV &pt : local_obj_pcloud->points) {
            pcl::PointXYZHSV global_pt =
                this->convert_to_global(pt);
            global_obj_pcloud->push_back(global_pt);
        }
        // pcl::transformPointCloud(*local_obj_pcloud, *global_obj_pcloud, m_lidar_map_tf);
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
    this->publish_map(m_local_header, "untracked", true, untracked_obs, m_untracked_map_pub,
                      untracked_labels);

    // EKF SLAM ("Probabilistic Robotics", Seb. Thrun, implementation, in one case removed robot
    // pose from the state and the respective motion updates, changed assignment algorithm)

    /*
    ******NOTES*******
    BECAUSE THE BUOY POSITION PREDICTIONS ARE NOT CORRELATED WITH ONE ANOTHER DIRECTLY, BUT JUST VIA
    THE ROBOT POSE THERE ARE TWO OPTIONS EITHER I DON'T INCLUDE THE ROBOT POSE, SO THEN THE
    CORRELATION BETWEEN BUOYS WILL BE ZERO SO I WILL JUST STORE ONE COVARIANCE MATRIX FOR EACH BUOY
    (2x2) WHICH WILL ALSO BE COMPUTATIONALLY EFFICIENT, BUT WON'T FIX THE POSITIONS OF OTHER BUOYS
    WHEN THE ROBOT FIXES ITS POSE USING THE POSITION OF THE OTHER BUOYS (MOSTLY DURING TURNS AS IT
    DRIFTS) TO DO THE LATTER I NEED TO FIND AN APPROPRIATE MOTION UPDATE SINCE I HAVE THE POSITION
    AND ORIENTATION USING THE ODOMETRY DATA SO I NEED TO JUST SET IT TO THAT POSE AND ADD GAUSSIAN
    NOISE (THAT FOR THE MOTION UPDATE) AND THEN THE MEASUREMENT UPDATE WILL UPDATE THE BUOYS
    POSITIONS APPROPRIATELY BASED ON THE NEW POSITION AND COVARIANCE AND HAVE THE ENTIRE
    (3N+2)x(3N+2) COVARIANCE MATRIX STORED, BUT WILL BE MORE COMPLICATED CODE-WISE AND SPEED-WISE
    THE LATTER WILL JUST KEEP THE CORRELATION BETWEEN THE OBSERVED OBSTACLES, SUCH THAT WHEN IT
    PERFORMS A MINOR FIX TO ONE OF THEM THAT MIGHT HAVE BEEN DUE TO TEMPORAY DRIFT, IT FIXES THE
    OTHERS AS WELL
    -->BECAUSE EKF IS BASED ON LINEAR UPDATES, JUST CREATE AN ARTIFICIAL LINEAR VELOCITY BY DIVIDING
    THE DISTANCE FROM THE LAST PREDICTED STATE TO THE NEW ODOMETRY DATA WITH TIME

    PUT THE ANGLE OF THE ROBOT IN THE PREDICTED STATE (SET ITS ANGULAR UNCERTAINTY PROPORTIONALLY TO
    ITS ANGULAR VELOCITY), TO SOLVE THE DRIFT ISSUE, HOPEFULLY FIX THE IMU PREDICTIONS AND THE LAG
    ISSUE WHEN TURNING BY ALIGNING IT WITH THE BUOYS ALSO PUT THE ROBOT POSE TO KEEP THE CORRELATION
    BETWEEN THE BUOY POSITIONS AND THEY DON'T DRIFT APART FROM ONE ANOTHER, ESPECIALLY WHEN THE
    ROBOT DOESN'T SEE MANY OF THEM BECAUSE THEY ARE BEHIND IT
    -->THE POSITION OF THE ROBOT WILL UPDATE BY JUST SETTING IT TO THE NEW ONE AND HAVING A NORMAL
    NOISE
    -->THE ORIENTATION OF THE ROBOT WILL BE UPDATED BY USING ITS ANGULAR VELOCITY, AS IT MAY DRIFT
    ANYWAYS, SO WE MIGHT AS WELL ALIGN IT
    */

    Eigen::Matrix<float, 2, 2> Q{
        {m_range_std, 0},
        {0, m_bearing_std},
    };
    std::vector<std::vector<float>> p;
    // RCLCPP_INFO(this->get_logger(), "COMPUTE WITH UNKNOWN CORRESPONDENCE");
    for (std::shared_ptr<ObjectCloud> det_obs : detected_obstacles) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) =
            local_to_range_bearing_signature(det_obs->local_centroid, det_obs->label);
        p.push_back(std::vector<float>());
        Eigen::Vector2f z_pred;
        Eigen::MatrixXf Psi;
        for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
            if (m_track_robot) {
                float d_x = m_state(3 + 2 * tracked_id) - m_nav_x;
                float d_y = m_state(3 + 2 * tracked_id + 1) - m_nav_y;
                float q = d_x * d_x + d_y * d_y;
                z_pred = Eigen::Vector2f(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
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

    // Assign each detection to a tracked or new object using the computed probabilities (actually
    // -logs?)
    std::vector<int> match(detected_obstacles.size(), -1);
    float min_p = 0;
    std::unordered_set<int> chosen_detected, chosen_tracked;
    while (min_p < m_new_obj_slam_thres) {
        min_p = m_new_obj_slam_thres;
        std::pair<int, int> best_match = std::make_pair(-1, -1);
        for (size_t i = 0; i < detected_obstacles.size(); i++) {
            if (chosen_detected.count(i))
                continue;
            for (int tracked_id = 0; tracked_id < m_num_obj; tracked_id++) {
                if (chosen_tracked.count(tracked_id) ||
                    m_tracked_obstacles[tracked_id]->label != detected_obstacles[i]->label)
                    continue;
                // RCLCPP_INFO(this->get_logger(), "P(%d, %d)=%lf", i, tracked_id,
                // p[i][tracked_id]);
                if (p[i][tracked_id] < min_p) {
                    best_match = std::make_pair(i, tracked_id);
                    min_p = p[i][tracked_id];
                }
            }
        }
        if (min_p < m_new_obj_slam_thres) {
            // RCLCPP_INFO(this->get_logger(), "MATCHING (%d, %d), with p: %lf", best_match.first,
            // best_match.second, p[best_match.first][best_match.second]);
            match[best_match.first] = best_match.second;
            chosen_tracked.insert(best_match.second);
            chosen_detected.insert(best_match.first);
        }
    }

    // Update vectors, now with known correspondence
    //  RCLCPP_INFO(this->get_logger(), "UPDATE WITH KNOWN CORRESPONDENCE");
    for (size_t i = 0; i < detected_obstacles.size(); i++) {
        float range, bearing;
        int signature;
        std::tie(range, bearing, signature) = local_to_range_bearing_signature(
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
                m_state.tail(2) = Eigen::Vector2f(m_nav_x, m_nav_y) +
                                  range * Eigen::Vector2f(std::cos(bearing + m_nav_heading),
                                                          std::sin(bearing + m_nav_heading));
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
        int tracked_id = match[i] > 0 ? match[i] : m_num_obj - 1;
        if (m_track_robot) {
            float d_x = m_state(3 + 2 * tracked_id) - m_nav_x;
            float d_y = m_state(3 + 2 * tracked_id + 1) - m_nav_y;
            float q = d_x * d_x + d_y * d_y;
            Eigen::Matrix<float, 2, 5> h{
                {-std::sqrt(q) * d_x, -std::sqrt(q) * d_y, 0, std::sqrt(q) * d_x,
                 std::sqrt(q) * d_y},
                {d_y, -d_x, -1, -d_y, d_x},
            };
            Eigen::Vector2f z_pred(std::sqrt(q), std::atan2(d_y, d_x) - m_nav_heading);
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

    // Update all (new and old, since they also have correlation with each other) objects global
    // positions (including the cloud, and then the local points respectively)
    pcl::PointXYZ p0(0, 0, 0);
    float avg_dist = 0;
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
        pcl::PointXYZ upd_loc_centr;
        upd_loc_centr = this->convert_to_local(upd_glob_centr);
        // RCLCPP_INFO(this->get_logger(), "ROBOT POSE: (%lf, %lf, %lf), TRACKED OBSTACLE %d GLOBAL
        // COORDS: (%lf, %lf), LOCAL COORDS: (%lf, %lf)", m_nav_x, m_nav_y, m_nav_heading,
        // m_tracked_obstacles[i]->id, upd_glob_centr.x, upd_glob_centr.y, upd_loc_centr.x,
        // upd_loc_centr.y);
        // move everything based on the difference between that and the previous assumed global
        // (then local) centroid
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        for (pcl::PointXYZHSV &global_pt : m_tracked_obstacles[i]->global_pcloud_ptr->points) {
            // TODO: Find a way to correctly transfer the z coordinate from the local detection using the transforms
            global_pt.x += upd_glob_centr.x - m_tracked_obstacles[i]->global_centroid.x;
            global_pt.y += upd_glob_centr.y - m_tracked_obstacles[i]->global_centroid.y;
            upd_local_obj_pcloud->push_back(
                this->convert_to_local(global_pt));
        }
        // pcl::transformPointCloud(*m_tracked_obstacles[i]->global_pcloud_ptr, *m_tracked_obstacles[i]->local_pcloud_ptr, m_map_lidar_tf);
        m_tracked_obstacles[i]->global_centroid = upd_glob_centr;
        m_tracked_obstacles[i]->local_centroid = upd_loc_centr;
        avg_dist += pcl::euclideanDistance(p0, upd_loc_centr) / ((float)m_tracked_obstacles.size());
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
            m_is_sim ? m_cam_model.project3dToPixel(
                           cv::Point3d(camera_point.y, camera_point.z, -camera_point.x))
                     : m_cam_model.project3dToPixel(
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
                // rclcpp::Time(msg->objects[0].time).seconds());
                m_tracked_obstacles[tracked_id]->time_dead =
                    rclcpp::Time(msg->objects[0].time) -
                    m_tracked_obstacles[tracked_id]->last_dead +
                    m_tracked_obstacles[tracked_id]->time_dead;
                // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d DEAD FOR %lf SECONDS, OBSTACLE DROP
                // THRESHOLD: %lf", tracked_id,
                // m_tracked_obstacles[tracked_id]->time_dead.seconds(), m_obstacle_drop_thresh);
                if (m_tracked_obstacles[tracked_id]->time_dead.seconds() >
                    m_obstacle_drop_thresh *
                        (pcl::euclideanDistance(p0,
                                                m_tracked_obstacles[tracked_id]->local_centroid) /
                         avg_dist) *
                        m_normalize_drop_dist) {
                    // RCLCPP_INFO(this->get_logger(), "OBSTACLE %d/%d DROPPED", tracked_id,
                    // m_num_obj);
                    to_remove.push_back(tracked_id);
                    continue;
                }
            }
            m_tracked_obstacles[tracked_id]->is_dead = true;
            m_tracked_obstacles[tracked_id]->last_dead = msg->objects[0].time;
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
        std::vector<std::shared_ptr<ObjectCloud>> new_obj;

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

    // Publish map with tracked obstacles
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> tracked_obs;
    std::vector<int> tracked_labels;
    for (std::shared_ptr<ObjectCloud> t_ob : m_tracked_obstacles) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tracked_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for (pcl::PointXYZHSV pt : t_ob->local_pcloud_ptr->points) {
            pcl::PointXYZRGB rgb_pt;
            pcl::PointXYZI i_pt;
            pcl::PointXYZHSVtoXYZRGB(pt, rgb_pt);
            pcl::PointXYZRGBtoXYZI(rgb_pt, i_pt);
            tracked_cloud->push_back(i_pt);
        }
        std::vector<int> ind(tracked_cloud->size());
        std::iota(std::begin(ind), std::end(ind), 0);
        std::shared_ptr<all_seaing_perception::Obstacle> tracked_ob(
            new all_seaing_perception::Obstacle(m_local_header, m_global_header, tracked_cloud, ind,
                                                t_ob->id, m_lidar_map_tf));
        tracked_labels.push_back(t_ob->label);
        tracked_obs.push_back(tracked_ob);
    }
    this->publish_map(m_local_header, "tracked", true, tracked_obs, m_tracked_map_pub,
                      tracked_labels);
    // RCLCPP_INFO(this->get_logger(), "AFTER TRACKED MAP PUBLISHING");
    // publish covariance ellipsoid markers to understand prediction uncertainty (& fine-tune &
    // debug)
    this->visualize_predictions();
}

void ObjectTrackingMap::publish_map(
    std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
    const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub,
    std::vector<int> labels) {

    // Create global header
    std_msgs::msg::Header global_header = std_msgs::msg::Header();
    global_header.frame_id = m_global_frame_id;
    global_header.stamp = local_header.stamp;

    all_seaing_interfaces::msg::ObstacleMap map_msg;
    map_msg.ns = ns;
    map_msg.local_header = local_header;
    map_msg.header = global_header;
    map_msg.is_labeled = is_labeled;
    for (size_t i = 0; i < map.size(); i++) {
        all_seaing_interfaces::msg::Obstacle det_obstacle;
        map[i]->to_ros_msg(det_obstacle);
        if (is_labeled) {
            det_obstacle.label = labels[i];
        }
        map_msg.obstacles.push_back(det_obstacle);
    }
    pub->publish(map_msg);
}

ObjectTrackingMap::~ObjectTrackingMap() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMap>());
    rclcpp::shutdown();
    return 0;
}
