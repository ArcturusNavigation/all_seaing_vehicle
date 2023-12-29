cmake_minimum_required(VERSION 3.8)
project(all_seaing_vehicle)

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
find_package(tf2 REQUIRED)

# ROS messages
find_package(protobuf_client_interfaces REQUIRED)
find_package(all_seaing_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)



#---------- ADD C++ EXECUTABLES ----------#

#---------- Competition-specific programs ----------#

#---------- Controller programs ----------#

#---------- Localization programs ----------#

#---------- Mapping programs ----------#

#---------- Navigation programs ----------#

#---------- Perception programs ----------#

#---------- Utils programs ----------#

# Node: state_reporter
add_executable(state_reporter src/utils/state_reporter.cpp)
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
add_executable(message_parser src/utils/message_parser.cpp)
ament_target_dependencies(message_parser
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)

# Node: message_sender
add_executable(message_sender src/utils/message_sender.cpp)
ament_target_dependencies(message_sender
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)

# Node: message_sender_goal
add_executable(message_sender_goal src/utils/message_sender_goal.cpp)
ament_target_dependencies(message_sender_goal
    rclcpp
    std_msgs
    geometry_msgs
    protobuf_client_interfaces
    all_seaing_interfaces
)



#---------- INSTALL C++ PROGRAMS ----------#

install(TARGETS
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
    src/competitions/vrx_2023/vrx_2023_acoustic_sender.py
    src/competitions/vrx_2023/vrx_2023_acoustic_sub.py
    src/competitions/vrx_2023/vrx_2023_pwm_sub.py
    src/competitions/vrx_2023/vrx_2023_stationkeeping.py
    src/competitions/vrx_2023/vrx_2023_task_sender.py
    src/competitions/vrx_2023/vrx_2023_waypoints.py

    # Controller programs
    src/controller/goal_publisher.py
    src/controller/sample_publisher.py
    src/controller/simple_controller.py
    src/controller/state_publisher.py
    src/controller/xdrive_controller.py

    # Localization progams

    # Mapping programs

    # Navigation programs
    src/navigation/pwm_sender.py

    # Perception programs
    src/perception/topic_to_image.py
    src/perception/yolov5_detector.py
    src/perception/yolov5_image.py

    # Utils programs

    DESTINATION lib/${PROJECT_NAME}
)



#---------- INSTALL DIRECTORIES ----------#

install(DIRECTORY
    launch
    models
    params
    urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
