#include "all_seaing_perception/object_tracking_map.hpp"

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map"){
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "odom");
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    RCLCPP_DEBUG(this->get_logger(), "OBSTACLE SEG THRESHOLD: %lf, DROP THRESHOLD: %lf", m_obstacle_seg_thresh, m_obstacle_drop_thresh);
    // Initialize navigation variables to 0
    m_nav_x = 0;
    m_nav_y = 0;
    m_nav_heading = 0;

    // Initialize publishers and subscribers
    m_untracked_map_pub =
        this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>("obstacle_map/refined_untracked", 10);
    m_tracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_tracked", 10);
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
    RCLCPP_DEBUG(this->get_logger(), "GOT CAMERA INFO");
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
    T new_point = point;//to keep color-related data
    double magnitude = std::hypot(point.x, point.y);
    double point_angle = std::atan2(point.y, point.x);
    new_point.x = nav_x + std::cos(nav_heading + point_angle) * magnitude;
    new_point.y = nav_y + std::sin(nav_heading + point_angle) * magnitude;
    new_point.z = 0;
    return new_point;
}

//TODO: Check it actually works, individually from the other parts
template <typename T>
T ObjectTrackingMap::convert_to_local(double nav_x, double nav_y, double nav_heading, T point) {
    T new_point = point;//to keep color-related data
    double diff_x = point.x - nav_x;
    double diff_y = point.y - nav_y;
    double magnitude = std::hypot(diff_x, diff_y);
    double point_angle = std::atan2(diff_y, diff_x);
    new_point.x = std::cos(point_angle - nav_heading) * magnitude;
    new_point.y = std::sin(point_angle - nav_heading) * magnitude;
    new_point.z = 0;
    return new_point;
}

void ObjectTrackingMap::object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg){
    if(msg->objects.size()==0) return;

    //compute global coordinates from camera frame point cloud detections
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

    /*
    OBJECT TRACKING:
    --Like in obstacle_detector.cpp, for each detected obstacle, find the closest (RMS point distance? or just centroids),
    same-color, yet unmatched, detected obstacle, and assign it to it --> if above distance threshold for all mark it as new
    --Update matched obstacles and add new ones (and assign them id)
    --From each tracked & unmatched obstacle, check if it's in the robot's FoV --> those are the only ones that may be detected at that point
    (subscribe to the camera_info topic and get the transform from LiDAR 3D to camera 2D plane and min/max image x,y,
    project the local centroid (from its global centroid and get_local()-->also update it in the tracked obstacles)
    of the obstacle onto the camera image plane and check if it's within the bounds, with some margin)
    --For those objects, update the time they have been undetected in the FoV total (add time from prev appearance, time_seen, if it was in FoV and undetected then too)
    --For unmatched obstacles that have been undetected in FoV for > some time threshold, remove them
    */

    // Match new obstacles with old ones, update them, and add new ones
    std::unordered_set<int> chosen_indices;
    for(std::shared_ptr<ObjectCloud> det_obs : detected_obstacles){
        float best_dist = m_obstacle_seg_thresh;
        int best_match = -1;
        for(int tracked_id = 0; tracked_id < m_tracked_obstacles.size(); tracked_id++){
            if (chosen_indices.count(tracked_id) || m_tracked_obstacles[tracked_id]->label != det_obs->label)
                continue;
            float curr_dist = pcl::euclideanDistance(m_tracked_obstacles[tracked_id]->global_centroid,
                                                     det_obs->global_centroid);
            if (curr_dist < best_dist) {
                best_match = tracked_id;
                best_dist = curr_dist;
            }
        }
        if(best_match >= 0){
            det_obs->id = m_tracked_obstacles[best_match]->id;
            m_tracked_obstacles[best_match] = det_obs;
            chosen_indices.insert(best_match);
        }else{
            det_obs->id = m_obstacle_id++;
            m_tracked_obstacles.push_back(det_obs);
        }
    }

    // Filter old obstacles
    for (int tracked_id = 0; tracked_id < m_tracked_obstacles.size(); tracked_id++) {
        if(chosen_indices.count(tracked_id)) continue;
        // Update local points
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        for(pcl::PointXYZHSV &global_pt : m_tracked_obstacles[tracked_id]->global_pcloud_ptr->points){ 
            upd_local_obj_pcloud->push_back(this->convert_to_local(m_nav_x, m_nav_y, m_nav_heading, global_pt));
        }
        m_tracked_obstacles[tracked_id]->update_loc_pcloud(upd_local_obj_pcloud);
        // Check if in FoV (which'll mean it's dead, at least temporarily)
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = m_tracked_obstacles[tracked_id]->local_centroid.x;
        lidar_point.y = m_tracked_obstacles[tracked_id]->local_centroid.y;
        lidar_point.z = m_tracked_obstacles[tracked_id]->local_centroid.z;
        geometry_msgs::msg::Point camera_point = lidar_point;//ALREADY IN THE SAME FRAME, WAS TRANSFORMED BEFORE BEING PUBLISHED BY bbox_project_pcloud.cpp
        cv::Point2d xy_rect = m_cam_model.project3dToPixel(
            cv::Point3d(camera_point.y, camera_point.z, -camera_point.x));
        RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d (%lf, %lf, %lf)->(%lf, %lf)", m_tracked_obstacles[tracked_id]->id, lidar_point.x, lidar_point.y, lidar_point.z, xy_rect.x, xy_rect.y);
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
            (xy_rect.y < m_cam_model.cameraInfo().height) && (lidar_point.x >= 0)) {
            // Dead
            if(m_tracked_obstacles[tracked_id]->is_dead){
                // Was also dead before, add time dead
                RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d TIME PERIOD FROM PREVIOUS DEAD: %lf - %lf", m_tracked_obstacles[tracked_id]->id, m_tracked_obstacles[tracked_id]->last_dead.seconds(), rclcpp::Time(msg->objects[0].time).seconds());
                m_tracked_obstacles[tracked_id]->time_dead = rclcpp::Time(msg->objects[0].time) - m_tracked_obstacles[tracked_id]->last_dead + m_tracked_obstacles[tracked_id]->time_dead;
                RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d DEAD FOR %lf SECONDS, OBSTACLE DROP THRESHOLD: %lf", m_tracked_obstacles[tracked_id]->id, m_tracked_obstacles[tracked_id]->time_dead.seconds(), m_obstacle_drop_thresh);
                if(m_tracked_obstacles[tracked_id]->time_dead.seconds() > m_obstacle_drop_thresh){
                    RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d DROPPED", m_tracked_obstacles[tracked_id]->id);
                    m_tracked_obstacles.erase(m_tracked_obstacles.begin()+tracked_id);
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