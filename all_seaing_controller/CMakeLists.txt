cmake_minimum_required(VERSION 3.8)
project(all_seaing_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(
    moos_to_controller
    ${PROJECT_NAME}/moos_to_controller.cpp
)

install(TARGETS
    moos_to_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/angle_printer.py
    ${PROJECT_NAME}/joy_to_control.py
    ${PROJECT_NAME}/odom_printer.py
    ${PROJECT_NAME}/sample_publisher.py
    ${PROJECT_NAME}/simple_controller.py
    ${PROJECT_NAME}/xdrive_controller.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
