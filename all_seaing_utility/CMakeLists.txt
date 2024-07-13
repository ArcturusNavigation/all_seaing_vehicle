cmake_minimum_required(VERSION 3.8)
project(all_seaing_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/onshore_node.py
    ${PROJECT_NAME}/replay_recordings.py
    ${PROJECT_NAME}/rosbag_to_mp4.py
    ${PROJECT_NAME}/rviz_testing_helper.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
