cmake_minimum_required(VERSION 3.8)
project(all_seaing_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(yaml-cpp REQUIRED)

ament_auto_add_executable(
    obstacle_detector
    ${PROJECT_NAME}/obstacle_detector.cpp
    ${PROJECT_NAME}/obstacle.cpp
)
ament_auto_add_executable(
    obstacle_bbox_overlay
    ${PROJECT_NAME}/obstacle_bbox_overlay.cpp
)
ament_auto_add_executable(
    point_cloud_filter
    ${PROJECT_NAME}/point_cloud_filter.cpp
)
ament_auto_add_executable(
    point_cloud_image_overlay
    ${PROJECT_NAME}/point_cloud_image_overlay.cpp
)
ament_auto_add_executable(
    obstacle_bbox_visualizer
    ${PROJECT_NAME}/obstacle_bbox_visualizer.cpp
)

target_link_libraries(obstacle_bbox_visualizer yaml-cpp)

install(TARGETS
    obstacle_detector
    obstacle_bbox_overlay
    point_cloud_filter
    point_cloud_image_overlay
    obstacle_bbox_visualizer
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/color_segmentation.py
    ${PROJECT_NAME}/yolov5_detector.py
    ${PROJECT_NAME}/yolov5_image.py
    ${PROJECT_NAME}/yolov8_node.py
    ${PROJECT_NAME}/image_splitter.py
    ${PROJECT_NAME}/perception_eval.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
