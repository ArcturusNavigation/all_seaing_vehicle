cmake_minimum_required(VERSION 3.8)
project(all_seaing_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/roboboat/delivery_server.py
    ${PROJECT_NAME}/roboboat/delivery_qual.py
    ${PROJECT_NAME}/roboboat/follow_buoy_path.py
    ${PROJECT_NAME}/roboboat/follow_buoy_pid.py
    ${PROJECT_NAME}/roboboat/run_tasks.py
    ${PROJECT_NAME}/roboboat/task_init.py
    ${PROJECT_NAME}/roboboat/docking.py
    ${PROJECT_NAME}/roboboat/docking_fallback.py
    ${PROJECT_NAME}/roboboat/speed_challenge.py
    ${PROJECT_NAME}/roboboat/speed_challenge_pid.py
    ${PROJECT_NAME}/roboboat/mechanism_navigation.py
    ${PROJECT_NAME}/roboboat/return_home.py
    ${PROJECT_NAME}/roboboat/harbor_alert.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_auto_package(INSTALL_TO_SHARE protobuf)
