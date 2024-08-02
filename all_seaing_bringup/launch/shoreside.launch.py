from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    keyboard_params = os.path.join(bringup_prefix, "config", "keyboard_controls.yaml")

    bag_path = LaunchConfiguration("bag_path")
    launch_rviz = LaunchConfiguration("launch_rviz")
    record_bag = LaunchConfiguration("record_bag")

    bag_path_launch_arg = DeclareLaunchArgument("bag_path", default_value=".")
    launch_rviz_launch_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"]
    )
    record_bag_launch_arg = DeclareLaunchArgument(
        "record_bag", default_value="true", choices=["true", "false"]
    )

    rviz_testing_helper_node = launch_ros.actions.Node(
        package="all_seaing_utility",
        executable="rviz_testing_helper.py",
        parameters=[{"bag_path": bag_path}],
        condition=IfCondition(record_bag),
    )

    keyboard_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard",
    )

    keyboard_to_joy_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard_to_joy.py",
        parameters=[{"config_file_name": keyboard_params}],
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
        package="all_seaing_utility",
        executable="onshore_node.py",
        output="screen",
    )

    return LaunchDescription(
        [
            bag_path_launch_arg,
            launch_rviz_launch_arg,
            record_bag_launch_arg,
            rviz_testing_helper_node,
            keyboard_node,
            keyboard_to_joy_node,
            rviz_node,
            onshore_node,
        ]
    )
