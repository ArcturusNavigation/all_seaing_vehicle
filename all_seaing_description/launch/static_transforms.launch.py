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
    zed_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "zed_camera_link",
        ],
        condition=UnlessCondition(LaunchConfiguration("indoors")),
    )

    lidar_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.07018073566954665",
            "--y",
            "-0.10612744990375929",
            "--z",
            "0.01631157058161572",
            "--qx",
            "0.7174633834551565",
            "--qy",
            "0.004810627144703515",
            "--qz",
            "-0.005718431558079253",
            "--qw",
            "0.6965561361498939",
            "--frame-id",
            "zed_left_camera_optical_frame",
            "--child-frame-id",
            "velodyne",
        ],
    )

    base_link_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.7071068",
            "--qw",
            "0.7071068",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
    )

    base_link_to_gps = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "1.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "gps_link",
        ],
    )

    rotate_rotated_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--qx",
            "0.0",
            "--qy",
            "0.0",
            "--qz",
            "-0.7071068",
            "--qw",
            "0.7071068",
            "--frame-id",
            "rotated_base_link",
            "--child-frame-id",
            "base_link",
        ],
    )

    return [
        # map_to_odom,
        lidar_to_camera,
        zed_to_base,
        base_link_to_imu,
        base_link_to_gps,
        # rotate_rotated_base_link
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
