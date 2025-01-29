from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import subprocess


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")
    use_lora = LaunchConfiguration("use_lora")

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
        parameters=[
            {"joy_x_scale": 1.0},
            {"joy_y_scale": -0.8},
            {"joy_ang_scale": -0.3},
        ],
        output="screen",
        condition=UnlessCondition(use_lora),
    )

    onshore_lora_controller = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_lora_controller.py",
        output="screen",
        condition=IfCondition(use_lora),
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    return LaunchDescription(
        [
            rviz_node,
            onshore_lora_controller,
            onshore_node,
            keyboard_ld,
        ]
    )

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_lora", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
