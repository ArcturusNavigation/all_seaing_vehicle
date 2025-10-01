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
    imu_filter_params = os.path.join(
        driver_prefix, "config", "imu_filter.yaml"
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
                    # {"imu/frame_id": "imu_link_accel"},
                    # {"global_position/frame_id": "gps_link"},
                    # {"global_position/tf/frame_id": "gps_world"},
                    # {"global_position/tf/child_frame_id": "gps_link"},
                    # {"local_position/frame_id": "base_link"},
                    # {"local_position/tf/frame_id": "odom"},
                    # {"local_position/tf/child_frame_id": "base_link"},
                ],
                remappings=[
                    # ("mavros/local_position/odom", "odometry/filtered") # to just use the internal EKF
                ],
                output="both",
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="gps_converter.py",
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="odom_reframe.py",
                parameters=[
                    {"target_child_frame_id": "imu_link_odom"}
                ],
                remappings=[
                    ("odom_topic", "/mavros/local_position/odom"),
                    ("new_odom_topic", "/mavros/local_position/odom/reframed")
                ]
            ),
            launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='imu_filter',
                output='screen',
                parameters=[imu_filter_params],
                remappings=[
                    ("imu/data_raw", "/mavros/imu/data"),
                    ("imu/mag", "/mavros/imu/mag"),
                    ("imu/data", "/mavros/imu/data/filtered"),
                ]
            ),
            launch_ros.actions.Node(
                package="all_seaing_driver",
                executable="imu_reframe.py",
                parameters=[
                    {"target_frame_id": "imu_link_accel"},
                    {"zero_g": True},
                    {"flip_gyro": True},
                ],
                remappings=[
                    ("imu_topic", "/mavros/imu/data/filtered"),
                    ("new_imu_topic", "/mavros/imu/data/reframed")
                ]
            ),
        ]
    )
