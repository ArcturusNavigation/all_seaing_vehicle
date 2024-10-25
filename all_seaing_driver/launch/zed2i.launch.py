from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("zed_wrapper"),
                        "/launch/zed_camera.launch.py",
                    ]
                ),
                launch_arguments={"camera_model": "zed2i"}.items(),
            )
        ]
    )
