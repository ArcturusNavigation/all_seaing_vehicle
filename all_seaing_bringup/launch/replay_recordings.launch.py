from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# this will run everything except the heartbeat, so you can directly Ctrl+C the heartbeat


def generate_launch_description():
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")
    bag_path = os.path.join(os.path.dirname(os.path.dirname(get_package_prefix("all_seaing_vehicle"))), "src", "all_seaing_vehicle", "all_seaing_ros2", "bag")

    return LaunchDescription([
        DeclareLaunchArgument("recording_name"),
        launch_ros.actions.Node(
            package="keyboard",
            executable="keyboard",
            name="keyboard"
        ),
        launch_ros.actions.Node(
            package="keyboard",
            executable="keyboard_to_joy.py",
            name="keyboard_to_joy",
            parameters=[{"config_file_name": os.path.join(all_seaing_prefix, "params", "keyboard_config.yaml")}]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(all_seaing_prefix, "params", "rviz_config.rviz")],
        ),
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="replay_recordings.py",
            parameters=[{"bag_path": bag_path, "recording_path": LaunchConfiguration("recording_name")}],
            on_exit=[
                EmitEvent(event=Shutdown(reason="recording player died"))
            ]
        )
    ])
