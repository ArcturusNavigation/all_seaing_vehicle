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
                    {"imu/frame_id": "imu_link"},
                    {"global_position/frame_id": "gps_link"},
                    # {"global_position/tf/frame_id": "gps_world"},
                    # {"global_position/tf/child_frame_id": "gps_link"},
                ],
                output="both",
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="gps_converter.py",
            ),
        ]
    )
