cmake_minimum_required(VERSION 3.8)
project(all_seaing_vehicle)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()



#---------- PACKAGE DEPENDENCIES ----------#

# Build system dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# General package dependencies
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)

# Transform dependencies
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# ROS messages
find_package(protobuf_client_interfaces REQUIRED)
find_package(all_seaing_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# PCL dependencies
find_package(pcl_ros)
find_package(pcl_conversions REQUIRED)

# Build
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(
    ${PCL_INCLUDE_DIRS} 
    /opt/ros/humble/include
    include
)



#---------- ADD C++ EXECUTABLES ----------#

#---------- Competition-specific programs ----------#

#---------- Controller programs ----------#

#---------- Localization programs ----------#

#---------- Mapping programs ----------#

#---------- Navigation programs ----------#

#---------- Perception programs ----------#

# Node: pointcloud_image_overlay
add_executable(pointcloud_image_overlay ${PROJECT_NAME}/perception/pointcloud_image_overlay.cpp)
target_link_libraries(pointcloud_image_overlay
    ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloud_image_overlay
    rclcpp
    std_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    image_geometry
    cv_bridge
    pcl_conversions
)

#---------- Utils programs ----------#

# Node: state_reporter
add_executable(state_reporter ${PROJECT_NAME}/utils/state_reporter.cpp)
ament_target_dependencies(state_reporter
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    all_seaing_interfaces
)

# Node: message_parser
add_executable(message_parser ${PROJECT_NAME}/utils/message_parser.cpp)
ament_target_dependencies(message_parser
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)

# Node: message_sender
add_executable(message_sender ${PROJECT_NAME}/utils/message_sender.cpp)
ament_target_dependencies(message_sender
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)

# Node: message_sender_goal
add_executable(message_sender_goal ${PROJECT_NAME}/utils/message_sender_goal.cpp)
ament_target_dependencies(message_sender_goal
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)



#---------- INSTALL C++ PROGRAMS ----------#

include_directories(include)

install(TARGETS
    pointcloud_image_overlay
    state_reporter
    message_parser
    message_sender
    message_sender_goal
    DESTINATION lib/${PROJECT_NAME}
)



#---------- INSTALL PYTHON PROGRAMS ----------#

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS

    # Competition-specific programs
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_acoustic_sender.py
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_acoustic_sub.py
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_pwm_sub.py
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_stationkeeping.py
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_task_sender.py
    ${PROJECT_NAME}/competitions/vrx_2023/vrx_2023_waypoints.py

    # Controller programs
    ${PROJECT_NAME}/controller/goal_publisher.py
    ${PROJECT_NAME}/controller/sample_publisher.py
    ${PROJECT_NAME}/controller/simple_controller.py
    ${PROJECT_NAME}/controller/state_publisher.py
    ${PROJECT_NAME}/controller/xdrive_controller.py

    # Localization progams

    # Mapping programs

    # Navigation programs
    ${PROJECT_NAME}/navigation/pwm_sender.py

    # Perception programs
    ${PROJECT_NAME}/perception/topic_to_image.py
    ${PROJECT_NAME}/perception/yolov5_detector.py
    ${PROJECT_NAME}/perception/yolov5_image.py

    # Utils programs

    DESTINATION lib/${PROJECT_NAME}
)



#---------- INSTALL MISC DIRECTORIES ----------#

install(DIRECTORY
    launch
    models
    params
    urdf
    DESTINATION share/${PROJECT_NAME}
)



ament_package()
