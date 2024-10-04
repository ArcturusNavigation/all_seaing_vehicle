from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import subprocess


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    keyboard_params = os.path.join(bringup_prefix, "config", "keyboard_controls.yaml")

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_rviz_launch_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"]
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

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(bringup_prefix, "rviz", "dashboard.rviz"),
        ],
        condition=IfCondition(launch_rviz),
    )

    onshore_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_node.py",
        output="screen",
    )

    return LaunchDescription(
        [
            launch_rviz_launch_arg,
            keyboard_node,
            keyboard_to_joy_node,
            rviz_node,
            onshore_node,
        ]
    )
