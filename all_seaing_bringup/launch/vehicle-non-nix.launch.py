from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    driver_prefix = get_package_share_directory("all_seaing_driver")

    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/zed2i.launch.py",
            ]
        )
    )

    return [
        zed_ld
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            OpaqueFunction(function=launch_setup),
        ]
    )
