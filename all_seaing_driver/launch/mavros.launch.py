from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros


def generate_launch_description():

    return LaunchDescription(
        [

            DeclareLaunchArgument(
                "port",
                default_value='/dev/ttyACM0',
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
            ExecuteProcess(
                cmd=['ros2', 'run', 'mavros', 'mav', 'sys', 'rate', '--all', '10'],
                output='screen',
            ),
        ]
    )
