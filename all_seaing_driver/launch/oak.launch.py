from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
import os
import yaml

def launch_setup(context, *args, **kwargs):
    driver_prefix = get_package_share_directory("all_seaing_driver")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
    oak_params = os.path.join(
        driver_prefix, "config", "oak.yaml"
    )

    back_left_oak_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, "launch", "camera.launch.py")
        ),
        launch_arguments={
            "name": "back_left_oak",
            "params_file": oak_params,
        }.items(),
    )

    back_right_oak_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_prefix, "launch", "camera.launch.py")
        ),
        launch_arguments={
            "name": "back_right_oak",
            "params_file": oak_params,
        }.items(),
    )

    return [
        back_left_oak_ld,
        back_right_oak_ld,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
