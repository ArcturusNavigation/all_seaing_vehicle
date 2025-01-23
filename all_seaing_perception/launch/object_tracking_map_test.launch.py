from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        remappings=[

        ],
        parameters=[
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
        ]
    )
    return LaunchDescription([
        object_tracking_map_node
    ])