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



#---------- INSTALL C++ PROGRAMS ----------#

install(TARGETS
    state_reporter
    DESTINATION lib/${PROJECT_NAME}
)



#---------- INSTALL PYTHON PROGRAMS ----------#

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS

    # Competition-specific programs

    # Controller programs
    src/controller/simple_controller.py

    # Localization progams

    # Mapping programs

    # Navigation programs
    src/navigation/pwm_sender.py

    # Perception programs
    src/perception/yolov5_detector.py
    src/perception/yolov5_image.py

    # Utils programs

    DESTINATION lib/${PROJECT_NAME}
)



#---------- INSTALL LAUNCH DIRECTORY ----------#

install(DIRECTORY
    launch
    params
    models
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
