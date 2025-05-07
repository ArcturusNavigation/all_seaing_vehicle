#include "all_seaing_perception/object_tracking_map_euclidean.hpp"

cv::Point2d custom_project(image_geometry::PinholeCameraModel cmodel, const cv::Point3d& xyz){
    cv::Point2d uv_rect;
    uv_rect.x = (cmodel.fx()*xyz.x + cmodel.Tx()) / xyz.z + cmodel.cx();
    uv_rect.y = (cmodel.fy()*xyz.y + cmodel.Ty()) / xyz.z + cmodel.cy();
    return uv_rect;
}

ObjectTrackingMapEuclidean::ObjectTrackingMapEuclidean() : Node("object_tracking_map_euclidean") {
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "map");
    this->declare_parameter<double>("obstacle_seg_thresh", 1.0);
    this->declare_parameter<double>("obstacle_drop_thresh", 1.0);
    this->declare_parameter<double>("odom_refresh_rate", 1000);

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();
    m_obstacle_seg_thresh = this->get_parameter("obstacle_seg_thresh").as_double();
    m_obstacle_drop_thresh = this->get_parameter("obstacle_drop_thresh").as_double();
    m_odom_refresh_rate = this->get_parameter("odom_refresh_rate").as_double();
    RCLCPP_DEBUG(this->get_logger(), "OBSTACLE SEG THRESHOLD: %lf, DROP THRESHOLD: %lf",
                 m_obstacle_seg_thresh, m_obstacle_drop_thresh);

    this->declare_parameter<bool>("is_sim", false);
    m_is_sim = this->get_parameter("is_sim").as_bool();

    this->declare_parameter<bool>("check_fov", false);
    m_check_fov = this->get_parameter("check_fov").as_bool();

    // Initialize publishers and subscribers
    m_untracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_untracked", 10);
    m_tracked_map_pub = this->create_publisher<all_seaing_interfaces::msg::ObstacleMap>(
        "obstacle_map/refined_tracked", 10);
    m_object_sub =
        this->create_subscription<all_seaing_interfaces::msg::LabeledObjectPointCloudArray>(
            "refined_object_point_clouds_segments", rclcpp::SensorDataQoS(),
            std::bind(&ObjectTrackingMapEuclidean::object_track_map_publish, this, std::placeholders::_1));
    m_image_intrinsics_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info_topic", 10,
        std::bind(&ObjectTrackingMapEuclidean::intrinsics_cb, this, std::placeholders::_1));

    // Initialize tf_listener pointer
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    odom_timer = this->create_wall_timer(
      std::chrono::duration<float>(((float)1.0)/m_odom_refresh_rate), std::bind(&ObjectTrackingMapEuclidean::odom_callback, this));

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
        // RCLCPP_DEBUG(this->get_logger(), "NEW POINT IN CREATED OBJECT POINT CLOUD: %lf, %lf,
        // %lf", global_pt.x, global_pt.y, global_pt.z);
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

void ObjectTrackingMapEuclidean::odom_callback() {
    if(!m_got_local_frame) return;

    //update odometry transforms
    //TODO: add a flag for each one that says if they succedeed, to know to continue or not
    m_map_lidar_tf = get_tf(m_global_frame_id, m_local_frame_id);
    m_lidar_map_tf = get_tf(m_local_frame_id, m_global_frame_id);
}

void ObjectTrackingMapEuclidean::intrinsics_cb(const sensor_msgs::msg::CameraInfo &info_msg) {
    RCLCPP_DEBUG(this->get_logger(), "GOT CAMERA INFO");
    m_cam_model.fromCameraInfo(info_msg);
}

geometry_msgs::msg::TransformStamped ObjectTrackingMapEuclidean::get_tf(const std::string &in_target_frame,
                                                               const std::string &in_src_frame) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = m_tf_buffer->lookupTransform(in_target_frame, in_src_frame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
    return tf;
}

template <typename T>
T ObjectTrackingMapEuclidean::convert_to_global(T point) {
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
T ObjectTrackingMapEuclidean::convert_to_local(T point) {
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

void ObjectTrackingMapEuclidean::object_track_map_publish(
    const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg) {
    if (msg->objects.size() == 0)
        return;
    // RCLCPP_DEBUG(this->get_logger(), "GOT DATA");

    // Set up headers and transforms
    // m_local_header = msg->objects[0].cloud.header;
    m_local_header = msg->header;
    m_global_header.frame_id = m_global_frame_id;
    m_global_header.stamp = m_local_header.stamp;
    m_local_frame_id = m_local_header.frame_id;
    m_got_local_frame = true;

    m_lidar_map_tf = get_tf(m_global_frame_id, m_local_header.frame_id);
    m_map_lidar_tf = get_tf(m_local_frame_id, m_global_frame_id);

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
        // RCLCPP_DEBUG(this->get_logger(), "BEFORE OBSTACLE CREATION");
        std::shared_ptr<ObjectCloud> obj_cloud(
            new ObjectCloud(obj.time, obj.label, local_obj_pcloud, global_obj_pcloud));
        detected_obstacles.push_back(obj_cloud);
    }
    // RCLCPP_DEBUG(this->get_logger(), "BEFORE UNTRACKED MAP PUBLISHING");
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
    this->publish_map(msg->objects[0].cloud.header, "untracked", true, untracked_obs,
                      m_untracked_map_pub, untracked_labels);
    // RCLCPP_DEBUG(this->get_logger(), "AFTER UNTRACKED MAP PUBLISHING");
    /*
    OBJECT TRACKING:
    --Like in obstacle_detector.cpp, for each detected obstacle, find the closest (RMS point
    distance? or just centroids), same-color, yet unmatched, detected obstacle, and assign it to it
    --> if above distance threshold for all mark it as new
    --Update matched obstacles and add new ones (and assign them id)
    --From each tracked & unmatched obstacle, check if it's in the robot's FoV --> those are the
    only ones that may be detected at that point (subscribe to the camera_info topic and get the
    transform from LiDAR 3D to camera 2D plane and min/max image x,y, project the local centroid
    (from its global centroid and get_local()-->also update it in the tracked obstacles) of the
    obstacle onto the camera image plane and check if it's within the bounds, with some margin)
    --For those objects, update the time they have been undetected in the FoV total (add time from
    prev appearance, time_seen, if it was in FoV and undetected then too)
    --For unmatched obstacles that have been undetected in FoV for > some time threshold, remove
    them
    */

    // Match new obstacles with old ones, update them, and add new ones
    std::unordered_set<int> chosen_indices;
    for (std::shared_ptr<ObjectCloud> det_obs : detected_obstacles) {
        float best_dist = m_obstacle_seg_thresh;
        int best_match = -1;
        for (int tracked_id = 0; tracked_id < m_tracked_obstacles.size(); tracked_id++) {
            if (chosen_indices.count(tracked_id) ||
                m_tracked_obstacles[tracked_id]->label != det_obs->label)
                continue;
            float curr_dist = pcl::euclideanDistance(
                m_tracked_obstacles[tracked_id]->global_centroid, det_obs->global_centroid);
            if (curr_dist < best_dist) {
                best_match = tracked_id;
                best_dist = curr_dist;
            }
        }
        if (best_match >= 0) {
            det_obs->id = m_tracked_obstacles[best_match]->id;
            m_tracked_obstacles[best_match] = det_obs;
            chosen_indices.insert(best_match);
        } else {
            det_obs->id = m_obstacle_id++;
            m_tracked_obstacles.push_back(det_obs);
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "AFTER NEW OBSTACLE MATCHING");
    // Filter old obstacles
    for (int tracked_id = 0; tracked_id < m_tracked_obstacles.size(); tracked_id++) {
        if (chosen_indices.count(tracked_id))
            continue;
        // Update local points
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr upd_local_obj_pcloud(
            new pcl::PointCloud<pcl::PointXYZHSV>);
        for (pcl::PointXYZHSV &global_pt :
             m_tracked_obstacles[tracked_id]->global_pcloud_ptr->points) {
            upd_local_obj_pcloud->push_back(
                this->convert_to_local(global_pt));
        }
        m_tracked_obstacles[tracked_id]->update_loc_pcloud(upd_local_obj_pcloud);
        // Check if in FoV (which'll mean it's dead, at least temporarily)
        geometry_msgs::msg::Point lidar_point;
        lidar_point.x = m_tracked_obstacles[tracked_id]->local_centroid.x;
        lidar_point.y = m_tracked_obstacles[tracked_id]->local_centroid.y;
        lidar_point.z = m_tracked_obstacles[tracked_id]->local_centroid.z;
        geometry_msgs::msg::Point camera_point =
            lidar_point; // ALREADY IN THE SAME FRAME, WAS TRANSFORMED BEFORE BEING PUBLISHED BY
                         // bbox_project_pcloud.cpp
        // tf2::doTransform<geometry_msgs::msg::Point>(lidar_point, camera_point, m_pc_cam_tf);
        cv::Point2d xy_rect =
            m_is_sim ? custom_project(m_cam_model,
                           cv::Point3d(camera_point.y, camera_point.z, -camera_point.x))
                     : custom_project(m_cam_model,
                           cv::Point3d(camera_point.x, camera_point.y, camera_point.z));
        ;
        RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d (%lf, %lf, %lf)->(%lf, %lf)",
                     m_tracked_obstacles[tracked_id]->id, lidar_point.x, lidar_point.y,
                     lidar_point.z, xy_rect.x, xy_rect.y);
        if ((xy_rect.x >= 0) && (xy_rect.x < m_cam_model.cameraInfo().width) && (xy_rect.y >= 0) &&
                (xy_rect.y < m_cam_model.cameraInfo().height) && (lidar_point.x >= 0) ||
            !m_check_fov) {
            // Dead
            if (m_tracked_obstacles[tracked_id]->is_dead) {
                // Was also dead before, add time dead
                RCLCPP_DEBUG(this->get_logger(),
                             "OBSTACLE ID %d TIME PERIOD FROM PREVIOUS DEAD: %lf - %lf",
                             m_tracked_obstacles[tracked_id]->id,
                             m_tracked_obstacles[tracked_id]->last_dead.seconds(),
                             rclcpp::Time(m_local_header.stamp).seconds());
                m_tracked_obstacles[tracked_id]->time_dead =
                    rclcpp::Time(m_local_header.stamp) -
                    m_tracked_obstacles[tracked_id]->last_dead +
                    m_tracked_obstacles[tracked_id]->time_dead;
                RCLCPP_DEBUG(this->get_logger(),
                             "OBSTACLE ID %d DEAD FOR %lf SECONDS, OBSTACLE DROP THRESHOLD: %lf",
                             m_tracked_obstacles[tracked_id]->id,
                             m_tracked_obstacles[tracked_id]->time_dead.seconds(),
                             m_obstacle_drop_thresh);
                if (m_tracked_obstacles[tracked_id]->time_dead.seconds() > m_obstacle_drop_thresh) {
                    RCLCPP_DEBUG(this->get_logger(), "OBSTACLE ID %d DROPPED",
                                 m_tracked_obstacles[tracked_id]->id);
                    m_tracked_obstacles.erase(m_tracked_obstacles.begin() + tracked_id);
                    tracked_id--;
                    continue;
                }
            }
            m_tracked_obstacles[tracked_id]->is_dead = true;
            m_tracked_obstacles[tracked_id]->last_dead = m_local_header.stamp;
        } else {
            m_tracked_obstacles[tracked_id]->is_dead = false;
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "AFTER OLD OBSTACLE FILTERING");
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
    this->publish_map(msg->objects[0].cloud.header, "tracked", true, tracked_obs, m_tracked_map_pub,
                      tracked_labels);
    RCLCPP_DEBUG(this->get_logger(), "AFTER TRACKED MAP PUBLISHING");
}

void ObjectTrackingMapEuclidean::publish_map(
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
    for (unsigned int i = 0; i < map.size(); i++) {
        all_seaing_interfaces::msg::Obstacle det_obstacle;
        map[i]->to_ros_msg(det_obstacle);
        if (is_labeled) {
            det_obstacle.label = labels[i];
        }
        map_msg.obstacles.push_back(det_obstacle);
    }
    pub->publish(map_msg);
}

ObjectTrackingMapEuclidean::~ObjectTrackingMapEuclidean() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMapEuclidean>());
    rclcpp::shutdown();
    return 0;
}
