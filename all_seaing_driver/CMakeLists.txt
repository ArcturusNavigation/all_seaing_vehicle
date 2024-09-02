cmake_minimum_required(VERSION 3.8)
project(all_seaing_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    ${PROJECT_NAME}/onshore_node.py
    ${PROJECT_NAME}/thrust_commander.py
    ${PROJECT_NAME}/webcam_publisher.py
    ${PROJECT_NAME}/webcam_splitter.py
    ${PROJECT_NAME}/zed2i_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
