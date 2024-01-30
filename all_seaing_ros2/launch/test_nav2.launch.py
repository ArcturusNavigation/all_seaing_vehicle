from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    nav2_prefix = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(
        get_package_share_directory("all_seaing_vehicle"), "params", "nav2_params.yaml"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_prefix, "/launch/tb3_simulation_launch.py"]
                ),
                launch_arguments={
                    "params_file": nav2_params,
                    "headless": "False",
                }.items(),
            ),
        ]
    )
