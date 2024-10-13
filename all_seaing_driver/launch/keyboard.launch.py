from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():
    keyboard_params = os.path.join(
        get_package_share_directory("all_seaing_driver"),
        "config",
        "keyboard_controls.yaml",
    )

    keyboard_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard",
    )

    keyboard_to_joy_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard_to_joy.py",
        parameters=[
            {"config_file_name": keyboard_params},
            {"sampling_frequency": 60},
        ],
    )

    return LaunchDescription(
        [
            keyboard_node,
            keyboard_to_joy_node,
        ]
    )
