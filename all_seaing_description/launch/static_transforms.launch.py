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

    odom_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "odom",
            "--child-frame-id",
            "base_link",
        ],
        # condition=UnlessCondition(LaunchConfiguration("indoors")),
    )

    # lidar_to_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "--x",
    #         "0.07018073566954665",
    #         "--y",
    #         "-0.10612744990375929",
    #         "--z",
    #         "0.01631157058161572",
    #         "--qx",
    #         "0.7174633834551565",
    #         "--qy",
    #         "0.004810627144703515",
    #         "--qz",
    #         "-0.005718431558079253",
    #         "--qw",
    #         "0.6965561361498939",
    #         "--frame-id",
    #         "zed_left_camera_optical_frame",
    #         "--child-frame-id",
    #         "velodyne",
    #     ],
    # )

    return [
        map_to_odom,
        # odom_to_base_link,
        # lidar_to_camera,
        # zed_to_base,
        # base_link_to_imu,
        # base_link_to_gps,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "indoors", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup)
        ]
    )
