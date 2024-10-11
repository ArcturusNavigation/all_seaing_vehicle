from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import yaml


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )
    locations_file = os.path.join(
        bringup_prefix, "config", "localization", "locations.yaml"
    )

    location = context.perform_substitution(LaunchConfiguration("location"))

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    with open(locations_file, 'r') as f:
        locations = yaml.safe_load(f)
    lat = locations[location]["lat"]
    lon = locations[location]["lon"]
    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/mavros/global_position/raw/fix")],
        parameters=[
            robot_localization_params,
            {"datum": [lat, lon, 0.0]},
        ],
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[{
            "in_sim": False,
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

    lidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/32e_points.launch.py",
            ]
        )
    )

    mavros_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/mavros.launch.py",
            ]
        ),
        launch_arguments={
            "port": "/dev/ttyACM0",
        }.items(),
    )

    ublox_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/ublox_gps.launch.py",
            ]
        ),
        launch_arguments={
            "port": "/dev/ttyACM1",
        }.items(),
    )

    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/zed2i.launch.py",
            ]
        )
    )

    return [
        ekf_node,
        navsat_node,
        controller_node,
        controller_server,
        waypoint_sender,
        thrust_commander_node,
        lidar_ld,
        mavros_ld,
        ublox_ld,
        zed_ld,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            OpaqueFunction(function=launch_setup),
        ]
    )
