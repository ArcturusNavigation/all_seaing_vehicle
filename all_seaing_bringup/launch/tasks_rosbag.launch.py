from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.actions import SetRemap
import os
import yaml
import xacro
import numpy as np


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    robot_urdf_file = os.path.join(
        description_prefix, "urdf", "fish_and_chips", "robot.urdf.xacro"
    )
    
    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )

    inc_color_buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "inc_color_buoy_label_mappings.yaml"
    )
    slam_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
    )
    
    locations_file = os.path.join(
        bringup_prefix, "config", "localization", "locations.yaml"
    )
    color_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "color_label_mappings.yaml"
    )
    color_buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "color_buoy_label_mappings.yaml"
    )
    buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
    )
    shape_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "shape_label_mappings.yaml"
    )
    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
    )

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_path.py",
        parameters=[
            {"is_sim": False},
            {"global_frame_id": "map"},
            {"color_label_mappings_file": buoy_label_mappings},
            {"duplicate_dist": 0.3},
            {"forward_dist": 1.5},
            {"inter_buoy_pair_dist": 0.5},
            {"buoy_pair_dist_thres": 0.2},
        ],
        remappings=[
            ("odometry/filtered", "odometry/tracked"),
        ],
    )

    follow_buoy_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_pid.py",
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": buoy_label_mappings},
            {"Kpid_x": [0.3, 0.0, 0.0]},
            {"Kpid_y": [0.3, 0.0, 0.0]},
            {"Kpid_theta": [0.2, 0.0, 0.0]},
            {"max_vel": [0.5, 0.5, 0.2]}
            # {"forward_speed": 0.5},
            # {"max_yaw": 0.001},
        ],
        remappings=[
            ("camera_info", "/zed/zed_node/rgb/camera_info"),
        ],
    )

    speed_challenge_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="speed_challenge_pid.py",
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": color_label_mappings},
            {"forward_speed": 1.2}
        ],
        remappings=[
            (
                "camera_info",
                "/zed/zed_node/rgb/camera_info"
            ),
            (
                "imu",
                "/mavros/imu/data"
            ),
        ],
    )

    docking_fallback = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="docking_fallback.py",
        parameters=[
            {"is_sim": False},
            {"shape_label_mappings_file": buoy_label_mappings},
            # {"shape_label_mappings_file": shape_label_mappings},
            # {"dock_width": 2.0},
            # {"dock_length": 11.0},
        ],
        remappings=[
            
        ],
    )

    control_mux = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="control_mux.py",
    )

    controller_server = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="controller_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
            {"Kpid_x": [0.3, 0.0, 0.0]},
            {"Kpid_y": [0.3, 0.0, 0.0]},
            {"Kpid_theta": [0.3, 0.0, 0.0]},
            {"max_vel": [1.5, 1.0, 0.1]},
        ],
        output="screen",
    )

    grid_map_generator = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="grid_map_generator.py",
        remappings=[
            ("odometry/filtered", "odometry/gps"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"timer_period": 0.4},
            {"grid_dim": [800, 800]},
            {"default_range": 60},
            {"grid_resolution": 0.1},
            {"dynamic_origin": True},
        ],
    )

    a_star_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="a_star_server",
        output="screen",
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
        ],
        output="screen",
    )

    navigation_server_nomap = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server_nomap.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
        ],
        output="screen",
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        # parameters=[{"is_sim": False}], # for IRL
        parameters=[{"is_sim": True}], # only for rosbag
    )

    delivery_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="delivery_server.py",
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    onshore_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_node.py",
        parameters=[
            {"joy_x_scale": 3.0},
            {"joy_y_scale": -2.0},
            {"joy_ang_scale": -1.5},
        ],
        output="screen",
    )

    rviz_waypoint_sender = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="rviz_waypoint_sender.py",
        parameters=[
            {"xy_threshold": 3.0},
            {"theta_threshold": 180.0},
            {"use_waypoint_client": False},
        ],
    )
    
    return [
        set_use_sim_time,
        controller_server,
        grid_map_generator,
        a_star_server,
        navigation_server,
        # navigation_server_nomap,
        run_tasks,
        task_init_server, 
        follow_buoy_path,
        # follow_buoy_pid,
        # delivery_server,
        # docking_fallback,
        keyboard_ld, # only for rosbag
        onshore_node, # only for rosbag
        rviz_waypoint_sender,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
