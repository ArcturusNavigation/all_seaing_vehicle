from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml
import os
import yaml
import xacro

def launch_setup(context, *args, **kwargs):
    driver_prefix = get_package_share_directory("all_seaing_driver")
    description_prefix = get_package_share_directory("all_seaing_description")
    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))
    # set_track_robot = launch_ros.actions.SetParameter(name='track_robot', value=LaunchConfiguration('use_slam')),

    slam_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
    )

    multicam_detection_merge_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="multicam_detection_merge.py",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings = [

        ],
        parameters = [
            {"enable_front": True},
            {"enable_back_left": True},
            {"enable_back_right": True},
            {"individual": False},
            {"approximate": False},
            {"delay": 0.1},
        ]
    )

    param_substitutions = {
        'track_robot': LaunchConfiguration('use_slam'),
    }

    configured_params = RewrittenYaml(
            source_file=slam_params,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("detections", "obstacle_map/local"),
            ("odometry/filtered", "odometry/gps"),
        ],
        parameters=[configured_params]
    )

    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms.launch.py",
            ]
        ),
    )

    map_to_odom = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
        condition=UnlessCondition(LaunchConfiguration("use_slam")),
    )

    return [
        # multicam_detection_merge_node,
        # set_track_robot, # if used should not set track robot from yaml
        object_tracking_map_node,
        # static_transforms_ld,
        map_to_odom,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument("location", default_value="boathouse"),
            DeclareLaunchArgument("use_slam", default_value="true", choices=["true", "false"]),
            OpaqueFunction(function=launch_setup),
        ]
    )
