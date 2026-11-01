cmake_minimum_required(VERSION 3.8)
project(all_seaing_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# C++ observability library (Python counterparts: all_seaing_common/run_logger.py and
# status_reporter.py): per-run logger (#501) and live status on /diagnostics (#480)
ament_auto_add_library(all_seaing_observability SHARED
    src/run_logger.cpp
    src/status_reporter.cpp
)

ament_python_install_package(${PROJECT_NAME})

ament_auto_package()

