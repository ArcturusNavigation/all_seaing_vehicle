cmake_minimum_required(VERSION 3.8)
project(all_seaing_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/control_mux.py
    ${PROJECT_NAME}/controller_server.py
    ${PROJECT_NAME}/xdrive_controller.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
