from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    robot_localization_params = os.path.join(
        bringup_prefix, "config", "robot_localization", "localize_seagrant.yaml"
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("/gps/fix", "/mavros/global_position/raw/fix")],
        parameters=[robot_localization_params],
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[{"in_sim": False}],
    )

    thrust_commander_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="thrust_commander.py",
        parameters=[
            {
                "frontright_port": 2,
                "frontleft_port": 3,
                "backright_port": 4,
                "backleft": 5,
            }
        ],
    )

    lidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("all_seaing_driver"),
                "/launch/32e_points.launch.py",
            ]
        )
    )

    mavros_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("all_seaing_driver"),
                "/launch/mavros.launch.py",
            ]
        )
    )

    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("all_seaing_driver"),
                "/launch/zed2i.launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            ekf_node,
            navsat_node,
            controller_node,
            thrust_commander_node,
            lidar_ld,
            mavros_ld,
            zed_ld,
        ]
    )
