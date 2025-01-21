#include "all_seaing_perception/object_tracking_map.hpp"

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map"){
    // Initialize parameters
    this->declare_parameter<std::string>("global_frame_id", "odom");

    // Initialize member variables from parameters
    m_global_frame_id = this->get_parameter("global_frame_id").as_string();

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
}

//copied from obstacle_detector.cpp
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

//copied & modified from obstacle.cpp
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

//TODO: Implement object tracking and tracked obstacle map publishing
void ObjectTrackingMap::object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray::ConstSharedPtr &msg){
    if(msg->objects.size()==0) return;
    std::vector<std::shared_ptr<ObjectTrackingMap::ObjectCloud>> raw_obstacles;
    for(all_seaing_interfaces::msg::LabeledObjectPointCloud obj : msg->objects){
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr local_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr global_obj_pcloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::fromROSMsg(obj.cloud, *local_obj_pcloud);
        for(pcl::PointXYZHSV &pt : local_obj_pcloud->points){
            pcl::PointXYZHSV global_pt = this->convert_to_global(m_nav_x, m_nav_y, m_nav_heading, pt);
            global_obj_pcloud->push_back(global_pt);
        }
        std::shared_ptr<ObjectTrackingMap::ObjectCloud> obj_cloud (new ObjectTrackingMap::ObjectCloud(obj.time, obj.label, local_obj_pcloud, global_obj_pcloud));
        raw_obstacles.push_back(obj_cloud);
    }
    //Make and publish untracked map
    std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> untracked_obs;
    for(std::shared_ptr<ObjectTrackingMap::ObjectCloud> raw_obs : raw_obstacles){
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(pcl::PointXYZHSV pt : raw_obs->local_pcloud_ptr->points){
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
                                                raw_obs->time_seen, m_nav_x, m_nav_y, m_nav_heading));
        untracked_obs.push_back(untracked_ob);
    }
    this->publish_map(msg->objects[0].cloud.header, "untracked", false, untracked_obs, m_untracked_map_pub);

    //Match new obstacles with old ones
}

void ObjectTrackingMap::publish_map(
    std_msgs::msg::Header local_header, std::string ns, bool is_labeled,
    const std::vector<std::shared_ptr<all_seaing_perception::Obstacle>> &map,
    rclcpp::Publisher<all_seaing_interfaces::msg::ObstacleMap>::SharedPtr pub) {

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
        all_seaing_interfaces::msg::Obstacle raw_obstacle;
        map[i]->to_ros_msg(local_header, global_header, raw_obstacle);
        map_msg.obstacles.push_back(raw_obstacle);
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