cmake_minimum_required(VERSION 3.8)
project(all_seaing_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt 8.1.1 EXACT REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/onshore_node.py
    ${PROJECT_NAME}/thrust_commander.py
    ${PROJECT_NAME}/webcam_publisher.py
    ${PROJECT_NAME}/zed2i_publisher.py
    ${PROJECT_NAME}/rover_lora_controller.py
    ${PROJECT_NAME}/rover_lora_heartbeat.py
    ${PROJECT_NAME}/rover_lora_combined.py
    ${PROJECT_NAME}/onshore_lora_controller.py
    ${PROJECT_NAME}/onshore_lora_heartbeat.py
    ${PROJECT_NAME}/onshore_lora_combined.py
    ${PROJECT_NAME}/gps_converter.py
    ${PROJECT_NAME}/driver_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
