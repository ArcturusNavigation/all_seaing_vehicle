cmake_minimum_required(VERSION 3.8)
project(all_seaing_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/roboboat/object_delivery.py
    ${PROJECT_NAME}/roboboat/follow_buoy_path.py
    ${PROJECT_NAME}/roboboat/run_tasks.py
    ${PROJECT_NAME}/roboboat/task_1.py
    ${PROJECT_NAME}/roboboat/task_2.py
    ${PROJECT_NAME}/vrx/acoustic_tracking/vrx_acoustic_tracking_sub.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE)
