cmake_minimum_required(VERSION 3.8)
project(all_seaing_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/rviz_waypoint_sender.py
    ${PROJECT_NAME}/utils.py
    ${PROJECT_NAME}/waypoint_publisher_test.py
    ${PROJECT_NAME}/a_star.py
    ${PROJECT_NAME}/map_publisher_test.py
    ${PROJECT_NAME}/navigation_server.py
    ${PROJECT_NAME}/waypoint_sender.py
    DESTINATION lib/${PROJECT_NAME}
)



# Optionally, install launch files (if you have any)
# install(DIRECTORY
#   launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_auto_package()
