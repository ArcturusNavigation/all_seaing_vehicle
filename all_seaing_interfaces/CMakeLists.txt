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
    "msg/ObstacleMap.msg"
    "msg/Obstacle.msg"
    "msg/ControlOption.msg"
    "msg/LabeledBoundingBox2DArray.msg"
    "msg/LabeledBoundingBox2D.msg"
    "msg/Heartbeat.msg"
    "msg/ASV2State.msg"
    "msg/BuoyPair.msg"
    "msg/BuoyPairArray.msg"
    "msg/Waypoint.msg"
    "msg/WaypointArray.msg"
    "msg/ObstacleCounter.msg"
    "msg/ObstacleCounterArray.msg"
    "msg/LabeledObjectPointCloud.msg"
    "msg/LabeledObjectPointCloudArray.msg"
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

ament_auto_package()
