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
        std::bind(&ObjectTrackingMap::object_track_publish, this, std::placeholders::_1));
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&ObjectTrackingMap::odom_callback, this, std::placeholders::_1));
}

//copied from obstacle_detector.cpp (same function, different class)
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

//TODO: Implement untracked (but labeled, since the data is provided that way) map publishing (just convert to required format using below copied map function and utility/message creating functions in obstacle.hpp)
//TODO: Implement object tracking and tracked map publishing
void object_track_map_publish(const all_seaing_interfaces::msg::LabeledObjectPointCloudArray &msg){

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