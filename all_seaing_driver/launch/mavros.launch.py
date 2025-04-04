from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value='/dev/ttyACM1',
            ),
            launch_ros.actions.Node(
                package="mavros",
                executable="mavros_node",
                parameters=[
                    {"fcu_url": LaunchConfiguration("port")},
                ],
                output="both",
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="gps_converter.py",
            ),
        ]
    )
