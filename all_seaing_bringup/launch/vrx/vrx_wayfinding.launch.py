from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")

    localize_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_sim.yaml"
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[localize_params],
    )

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/wamv/sensors/gps/gps/fix")],
        parameters=[localize_params],
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[{"in_sim": True}],
    )

    onshore_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_node.py",
        output="screen",
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
        launch_arguments={
            "world": "wayfinding_task",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
            "extra_gz_args": "-v 0",
        }.items(),
    )

    return LaunchDescription(
        [
            ekf_node,
            navsat_node,
            controller_node,
            onshore_node,
            keyboard_ld,
            sim_ld,
        ]
    )
