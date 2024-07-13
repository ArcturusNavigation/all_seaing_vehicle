cmake_minimum_required(VERSION 3.8)
project(all_seaing_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
    nav_state_reporter
    ${PROJECT_NAME}/nav_state_reporter.cpp
)
ament_auto_add_executable(
    stationkeeping_goal_sender
    ${PROJECT_NAME}/stationkeeping_goal_sender.cpp
)
ament_auto_add_executable(
    stationkeeping_state_reporter
    ${PROJECT_NAME}/stationkeeping_state_reporter.cpp
)

install(TARGETS
    nav_state_reporter
    stationkeeping_goal_sender
    stationkeeping_state_reporter
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/buoy_pair_finder.py
    ${PROJECT_NAME}/waypoint_sender.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
