cmake_minimum_required(VERSION 3.8)
project(all_seaing_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)
add_library(${PROJECT_NAME} SHARED
    src/obstacle_map_display.cpp
)
ament_target_dependencies(${PROJECT_NAME}
    pluginlib
    rviz_common
    rviz_default_plugins
    all_seaing_interfaces
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "OBJECT_DETECTION_PLUGINS_BUILDING_LIBRARY")
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-useless-cast
        -Wno-old-style-cast -Wno-double-promotion -Wno-pedantic -Wno-unused-parameter -Wno-overloaded-virtual)

install(TARGETS ${PROJECT_NAME} 
    EXPORT export_all_seaing_rviz_plugins
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_all_seaing_rviz_plugins)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_auto_package(INSTALL_TO_SHARE plugins_description.xml)