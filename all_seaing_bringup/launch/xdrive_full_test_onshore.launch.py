from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
import launch_ros
import os

# this will run everything needed to run on the onshore computer


def generate_launch_description():
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")
    bag_path = os.path.join(os.path.dirname(os.path.dirname(get_package_prefix("all_seaing_vehicle"))), "src", "all_seaing_vehicle", "all_seaing_ros2", "bag")

    return LaunchDescription([
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="rviz_testing_helper.py",
            name="rviz_testing_helper",
            parameters=[{"bag_path": bag_path}]
        ),
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
            arguments=["-d", os.path.join(all_seaing_prefix, "params", "rviz_config.rviz")]
        ),
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="onshore_node.py",
            name="onshore_node",
            output="screen"
        )
    ])
