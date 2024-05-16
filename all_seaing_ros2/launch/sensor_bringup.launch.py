import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Velodyne LiDAR
    lidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(
                "all_seaing_vehicle"
            ),
            "/launch/32e_points.launch.py"
        ])
    )

    # MAVROS
    mavros_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(
                "all_seaing_vehicle"
            ),
            "/launch/mavros.launch.py"
        ])
    )

    # ZED camera
    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(
                "zed_wrapper"
            ),
            "/launch/obsolete/zed2i.launch.py"
        ])
    )

    return LaunchDescription([
        lidar_ld,
        mavros_ld,
        zed_ld
    ])
