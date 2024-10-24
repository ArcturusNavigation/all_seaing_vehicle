from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import subprocess


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_rviz_launch_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"]
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
        parameters=[
            {"joy_x_scale": 2.0},
            {"joy_y_scale": -1.0},
            {"joy_ang_scale": -0.8},
        ],
    )

    onshore_lora_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_lora_controller.py",
        output="screen"
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    return LaunchDescription(
        [
            launch_rviz_launch_arg,
            rviz_node,
            onshore_node,
            #onshore_lora_node,
            #keyboard_ld,
        ]
    )
