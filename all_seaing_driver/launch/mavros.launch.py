from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    driver_prefix = get_package_share_directory("all_seaing_driver")
    mavros_params = os.path.join(
        driver_prefix, "config", "mavros.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value='/dev/ttyACM0',
            ),
            launch_ros.actions.Node(
                package="mavros",
                executable="mavros_node",
                parameters=[mavros_params,
                    # {"plugin_blacklist": ["*"]},
                    # {"plugin_whitelist": ["actuator_control", "command", "ftp", "global_position", "imu_pub", "setpoint_velocity"]},
                    {"fcu_url": LaunchConfiguration("port")},
                    # {"imu/frame_id": "imu_link"},
                    # {"global_position/frame_id": "gps_link"},
                    # {"global_position/tf/frame_id": "gps_world"},
                    # {"global_position/tf/child_frame_id": "gps_link"},
                    # {"local_position/frame_id": "base_link"},
                    # {"local_position/tf/frame_id": "odom"},
                    # {"local_position/tf/child_frame_id": "base_link"},
                ],
                output="both",
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="gps_converter.py",
            ),
        ]
    )
