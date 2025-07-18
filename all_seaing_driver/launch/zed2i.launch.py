from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

import os


def generate_launch_description():
    zed_params = os.path.join(
        get_package_share_directory("all_seaing_driver"),
        "config",
        "zed2i.yaml",
    )

    return LaunchDescription(
        [
            GroupAction(
                actions=[
                    SetRemap(src='/tf',dst='/tf_trash'),
                    SetRemap(src='/tf_static',dst='/tf_static_trash'),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                get_package_share_directory("zed_wrapper"),
                                "/launch/zed_camera.launch.py",
                            ]
                        ),
                        launch_arguments={
                            "camera_model": "zed2i",
                            "ros_params_override_path": zed_params,
                            "use_sim_time": "false",
                            # "publish_urdf": "false",
                        }.items(),
                    )
                ]
            )
        ]
    )
