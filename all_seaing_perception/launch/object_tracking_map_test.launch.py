from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
        ],
        parameters=[
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
            {"range_uncertainty": 1.0},
            {"bearing_uncertainty": 1.0},
            {"new_object_slam_threshold": 1.0}
        ]
    )
    return LaunchDescription([
        object_tracking_map_node
    ])