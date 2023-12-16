cmake_minimum_required(VERSION 3.8)
project(all_seaing_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament package dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ros client libraries
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ros messages
find_package(all_seaing_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# install the launch directory
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    src/controller/pwm_sender.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
