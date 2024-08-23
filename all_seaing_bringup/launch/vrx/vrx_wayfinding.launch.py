from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():

    vrx_gz_prefix = get_package_share_directory("vrx_gz")
    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    localize_params = os.path.join(
        bringup_prefix, "config", "robot_localization", "localize_sim.yaml"
    )
    keyboard_params = os.path.join(bringup_prefix, "config", "keyboard_controls.yaml")

    state_reporter_node = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="nav_state_reporter",
        remappings=[
            ("imu/data", "/wamv/sensors/imu/imu/data"),
            ("gps/fix", "/wamv/sensors/gps/gps/fix"),
        ],
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

    keyboard_node = launch_ros.actions.Node(package="keyboard", executable="keyboard")

    keyboard_to_joy_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard_to_joy.py",
        parameters=[{"config_file_name": keyboard_params}],
    )

    waypoint_sender_node = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="waypoint_sender.py",
        remappings=[("waypoints", "/vrx/wayfinding/waypoints")],
        parameters=[
            {"use_pose_array": True},
            {"use_gps": True},
        ],
    )

    protobuf_client_node = launch_ros.actions.Node(
        package="protobuf_client",
        executable="protobuf_client_node",
    )

    moos_to_controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="moos_to_controller",
    )

    onshore_node = launch_ros.actions.Node(
        package="all_seaing_utility",
        executable="onshore_node.py",
        output="screen",
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
            state_reporter_node,
            ekf_node,
            navsat_node,
            controller_node,
            keyboard_node,
            keyboard_to_joy_node,
            waypoint_sender_node,
            protobuf_client_node,
            moos_to_controller_node,
            onshore_node,
            sim_ld,
        ]
    )
