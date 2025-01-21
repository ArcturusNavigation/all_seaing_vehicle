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
            "0.0613202",
            "--y",
            "-0.22164122",
            "--z",
            "-0.12865914",
            "--qx",
            "0.725860918",
            "--qy",
            "0.025729045",
            "--qz",
            "-0.02346689",
            "--qw",
            "0.68695942",
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
