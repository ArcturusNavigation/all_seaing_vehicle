from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    keyboard_params = os.path.join(bringup_prefix, "config", "keyboard_controls.yaml")

    bag_path = LaunchConfiguration("bag_path")
    recording_name = LaunchConfiguration("recording_name")

    bag_path_launch_arg = DeclareLaunchArgument("bag_path", default_value=".")
    recording_name_launch_arg = DeclareLaunchArgument("recording_name")

    keyboard_node = (
        launch_ros.actions.Node(
            package="keyboard", executable="keyboard", name="keyboard"
        ),
    )

    keyboard_to_joy_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard_to_joy.py",
        name="keyboard_to_joy",
        parameters=[{"config_file_name": keyboard_params}],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(bringup_prefix, "config", "dashboard.rviz"),
        ],
    )

    replay_recordings_node = (
        launch_ros.actions.Node(
            package="all_seaing_utility",
            executable="replay_recordings.py",
            parameters=[
                {
                    "bag_path": bag_path,
                    "recording_path": recording_name,
                }
            ],
            on_exit=[EmitEvent(event=Shutdown(reason="recording player died"))],
        ),
    )

    return LaunchDescription(
        [
            bag_path_launch_arg,
            recording_name_launch_arg,
            keyboard_node,
            keyboard_to_joy_node,
            rviz_node,
            replay_recordings_node,
        ]
    )
