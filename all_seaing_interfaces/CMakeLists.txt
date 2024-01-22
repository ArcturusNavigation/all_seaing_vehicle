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

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ASVState.msg"
    "msg/ControlMessage.msg"
    "msg/ObstacleFeature.msg"
    "msg/GoalState.msg"
    "msg/LabeledBoundingBox2D.msg"
    "msg/LabeledBoundingBox2DArray.msg"
    DEPENDENCIES std_msgs
)

ament_package()
