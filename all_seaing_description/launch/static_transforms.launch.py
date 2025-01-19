from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    lidar_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.02",
            "--y",
            "-0.20",
            "--z",
            "-0.15",
            "--qx",
            "0.7071068",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "0.7071068",
            "--frame-id",
            "zed_left_camera_optical_frame",
            "--child-frame-id",
            "velodyne",
        ],
    )

    return [
        lidar_to_camera,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
