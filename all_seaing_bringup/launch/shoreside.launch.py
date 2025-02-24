from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

import launch_ros
import os
import subprocess


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")
    comms = LaunchConfiguration("comms")

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
        condition=IfCondition(PythonExpression(["'", comms, "' == 'wifi'"])),
    )

    onshore_lora_controller = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_lora_controller.py",
        output="screen",
        condition=IfCondition(PythonExpression(["'", comms, "' == 'lora'"])),
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
        condition=UnlessCondition(use_lora),
    )

    return [
        rviz_node,
        onshore_lora_controller,
        onshore_node,
        keyboard_ld,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "comms", default_value="wifi", choices=["wifi", "lora", "custom"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
