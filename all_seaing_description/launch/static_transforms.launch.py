from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        condition=UnlessCondition(LaunchConfiguration("indoors")),
    )

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
        map_to_odom,
        lidar_to_camera,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "indoors", default_value="true", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
