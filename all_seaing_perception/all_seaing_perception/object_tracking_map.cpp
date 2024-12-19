#include "all_seaing_perception/object_tracking_map.hpp"

ObjectTrackingMap::ObjectTrackingMap() : Node("object_tracking_map"){

}

ObjectTrackingMap::~ObjectTrackingMap() {}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectTrackingMap>());
    rclcpp::shutdown();
    return 0;
}