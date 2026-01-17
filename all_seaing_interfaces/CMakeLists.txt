cmake_minimum_required(VERSION 3.8)
project(all_seaing_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/FollowPath.action"
    "action/Waypoint.action"
    "action/Task.action"
    "action/Search.action"
    "msg/ObstacleMap.msg"
    "msg/Obstacle.msg"
    "msg/ControlOption.msg"
    "msg/LabeledBoundingBox2DArray.msg"
    "msg/LabeledBoundingBox2D.msg"
    "msg/Heartbeat.msg"
    "msg/ObstacleCounter.msg"
    "msg/ObstacleCounterArray.msg"
    "msg/LabeledObjectPointCloud.msg"
    "msg/LabeledObjectPointCloudArray.msg"
    "msg/LabeledObjectPlane.msg"
    "msg/LabeledObjectPlaneArray.msg"
    "msg/KeyboardButton.msg"
    "srv/CommandAdj.srv"
    "srv/CommandFan.srv"
    "srv/CommandServo.srv"
    "srv/GetEstopStatus.srv"
    "srv/AcknowledgeLowBattery.srv"
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

ament_auto_package()
