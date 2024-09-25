from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "robot_localization", "localize_real.yaml"
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/mavros/global_position/raw/fix")],
        parameters=[robot_localization_params],
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[{
            "boat_length": 0.7112,
            "boat_width": 0.2540,
            "min_output": 1100.0,
            "max_output": 1900.0,
        }],
    )

    thrust_commander_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="thrust_commander.py",
        parameters=[
            {
                "front_right_port": 2,
                "front_left_port": 3,
                "back_right_port": 4,
                "back_left_port": 5,
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

    controller_server = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="controller_server.py",
        output="screen",
    )

    waypoint_sender = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="waypoint_sender.py",
        parameters=[
            {"xy_threshold": 1.0},
            {"theta_threshold": 5.0},
        ],
        output="screen",
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
            controller_server,
            waypoint_sender,
            thrust_commander_node,
            lidar_ld,
            mavros_ld,
            zed_ld,
        ]
    )
