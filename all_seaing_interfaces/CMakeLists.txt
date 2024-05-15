cmake_minimum_required(VERSION 3.8)
project(all_seaing_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# ROS messages
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ASVState.msg"
    "msg/CloudClusterArray.msg"
    "msg/CloudCluster.msg"
    "msg/ControlMessage.msg"
    "msg/GoalState.msg"
    "msg/LabeledBoundingBox2DArray.msg"
    "msg/LabeledBoundingBox2D.msg"
    "msg/Heartbeat.msg"
    "msg/ASV2State.msg"
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()
