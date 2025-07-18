cmake_minimum_required(VERSION 3.8)
project(all_seaing_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(yaml-cpp REQUIRED)

find_package(PCL 1.12 REQUIRED COMPONENTS common visualization io)

find_package(Eigen3 3.4 REQUIRED)

ament_auto_add_executable(
    obstacle_detector
    ${PROJECT_NAME}/obstacle_detector.cpp
    ${PROJECT_NAME}/obstacle.cpp
    ${PROJECT_NAME}/perception_utilities.cpp
)

ament_auto_add_executable(
    point_cloud_filter
    ${PROJECT_NAME}/point_cloud_filter.cpp
)

ament_auto_add_executable(
    bbox_project_pcloud
    ${PROJECT_NAME}/bbox_project_pcloud.cpp
    ${PROJECT_NAME}/perception_utilities.cpp
    ${PROJECT_NAME}/obstacle.cpp
)
ament_auto_add_executable(
    object_tracking_map
    ${PROJECT_NAME}/object_tracking_map.cpp
    ${PROJECT_NAME}/obstacle.cpp
    ${PROJECT_NAME}/object_tracking_shared.cpp
    ${PROJECT_NAME}/perception_utilities.cpp
    ${PROJECT_NAME}/Hungarian.cpp
)

target_link_libraries(bbox_project_pcloud yaml-cpp ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})
target_link_libraries(object_tracking_map yaml-cpp Eigen3::Eigen ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

install(TARGETS
    obstacle_detector
    point_cloud_filter
    bbox_project_pcloud
    object_tracking_map
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/color_segmentation.py
    ${PROJECT_NAME}/color_segmentation_ycrcb.py
    ${PROJECT_NAME}/yolov8_node.py
    ${PROJECT_NAME}/perception_eval.py
    ${PROJECT_NAME}/multicam_detection_merge.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch models)
