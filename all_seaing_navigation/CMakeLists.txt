cmake_minimum_required(VERSION 3.8)
project(all_seaing_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
    a_star_server
    ${PROJECT_NAME}/a_star_server.cpp
)

install(TARGETS
    a_star_server
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/map_publisher_test.py
    ${PROJECT_NAME}/grid_map_generator.py
    ${PROJECT_NAME}/navigation_server.py
    ${PROJECT_NAME}/navigation_server_nomap.py
    ${PROJECT_NAME}/navigation_server_tangent.py
    ${PROJECT_NAME}/rviz_waypoint_sender.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
