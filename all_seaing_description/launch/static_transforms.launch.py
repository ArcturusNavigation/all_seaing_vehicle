from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    lidar_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # arguments=[
        #     "--x",
        #     "-0.05170783940451991",
        #     "--y",
        #     "-0.05187798511171696",
        #     "--z",
        #     "-0.39700142732955157",
        #     "--qx",
        #     "-0.658623187450687",
        #     "--qy",
        #     "0.013786635156961146",
        #     "--qz",
        #     "0.004802448059092996",
        #     "--qw",
        #     "0.7523312848313473",
        #     "--frame-id",
        #     "velodyne",
        #     "--child-frame-id",
        #     "zed_left_camera_optical_frame",
        # ],

        arguments=[
            "--x",
            "0.04037162245794776",
            "--y",
            "-0.38782290876795594",
            "--z",
            "0.104588158181082",
            "--qx",
            "-0.658623187450687",
            "--qy",
            "0.013786635156961146",
            "--qz",
            "0.004802448059092996",
            "--qw",
            "-0.7523312848313473",
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
