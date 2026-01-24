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
    task_locations = os.path.join(
        bringup_prefix, "config", "course", "task_locations_real.yaml"
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
    all_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "all_label_mappings.yaml"
    )
    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )

    use_waypoint_client = LaunchConfiguration("use_waypoint_client")

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
        parameters=[
            {"is_sim": False},
            {"red_left": True},
            {"global_frame_id": "map"},
            # {"color_label_mappings_file": buoy_label_mappings},
            {"color_label_mappings_file": all_label_mappings},
            {"task_locations_file": task_locations},
            {"gate_dist_thres": 25.0},
            {"circling_buoy_dist_thres": 25.0},
            {"max_inter_gate_dist": 25.0},
            {"max_gate_pair_dist": 25.0},
            {"duplicate_dist": 0.3},
            {"inter_buoy_pair_dist": 0.5},
            {"buoy_pair_dist_thres": 0.2},
        ]
    )

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_path.py",
        parameters=[
            {"is_sim": False},
            {"red_left": True},
            {"global_frame_id": "map"},
            # {"color_label_mappings_file": buoy_label_mappings},
            {"color_label_mappings_file": all_label_mappings},
            {"search_task_radius": 50.0},
            {"gate_dist_thres": 25.0},
            {"circling_buoy_dist_thres": 25.0},
            {"max_inter_gate_dist": 25.0},
            {"max_gate_pair_dist": 25.0},
            {"duplicate_dist": 0.3},
            {"forward_dist": 1.5},
            {"inter_buoy_pair_dist": 0.5},
            {"buoy_pair_dist_thres": 0.2},
            {"xy_threshold": 0.8},
            {"thresh_dist": 0.8},
            {"choose_every": 25},
            {"turn_offset": 3.0},
            {"beacon_probe_dist": 4.0},
            {"midpoint_pair_forward_dist": 0.5},
            {"adaptive_distance": 0.2},
            {"max_turn_vel": [1.5, 0.0, 0.2]},
            {"turn_pid": [0.4,0.0,0.0]},
        ],
        remappings=[
            ("odometry/filtered", "odometry/tracked"),
        ],
    )

    speed_challenge = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="speed_challenge.py",
        parameters=[
            {"is_sim": False},
            {"red_left": True},
            # {"color_label_mappings_file": buoy_label_mappings},
            {"color_label_mappings_file": all_label_mappings},
            {"robot_frame_id": "base_link"},
            {"search_task_radius": 50.0},
            {"gate_dist_thres": 40.0},
            {"beacon_dist_thres": 15.0},
            {"circling_buoy_dist_thres": 40.0},
            {"max_inter_gate_dist": 25.0},
            {"turn_offset": 1.5},
            {"choose_every": 5},
            {"probe_distance": 30},
            {"xy_threshold": 1.0},
            {"duplicate_dist": 0.3},
            {"init_gate_dist": 0.5},
            {"gate_dist_back": 2.0},
            {"adaptive_distance": 0.2},
            {"max_turn_vel": [1.5, 0.0, 0.2]},
            {"turn_pid": [0.4,0.0,0.0]},
        ],
        remappings=[
            ("odometry/filtered", "odometry/tracked"),
        ]
    )

    docking = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="docking.py",
        parameters=[
            {"is_sim": False},
            # {"shape_label_mappings_file": buoy_label_mappings},
            # {"shape_label_mappings_file": shape_label_mappings},
            {"shape_label_mappings_file": all_label_mappings},
            {"robot_frame_id": "base_link"},
            {"search_task_radius": 50.0},
            {"dock_width": 2.0},
            {"dock_length": 7.0},
            {"wpt_banner_dist": 8.0},
            {"navigation_dist_thres": 10.0},
            # {"update_slot_dist_thres": 3.0},
            {"docked_xy_thres": 0.5},
            {"duplicate_dist": 0.5},
            {"choose_every": 10},
            {"Kpid_x": [2.0, 0.0, 0.5]},
            {"Kpid_y": [2.0, 0.0, 0.5]},
            {"Kpid_theta": [0.3, 0.0, 1.0]},
            {"max_vel": [0.7, 0.7, 0.2]},
            # {"avoid_max_dist": 1.5},
            # {"avoid_vel_coeff": 2.0},
            {"avoid_max_dist": 5.0},
            {"avoid_vel_coeff": 1.0},
            {"rot_avoid_vel_coeff": 1.0},
            {"avoid_rot_vel_mag": True},
        ],
        remappings=[
            
        ],
    )

    mechanism_navigation = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="mechanism_navigation.py",
        parameters=[
            {"is_sim": False},
            # {"shape_label_mappings_file": buoy_label_mappings},
            # {"shape_label_mappings_file": shape_label_mappings},
            {"shape_label_mappings_file": all_label_mappings},
            {"robot_frame_id": "base_link"},
            {"search_task_radius": 50.0},
            {"wpt_banner_dist": 3.0},
            {"navigation_dist_thres": 5.0},
            # {"update_target_dist_thres": 3.0},
            {"shooting_xy_thres": 0.8},
            {"shooting_theta_thres": 10.0},
            {"duplicate_dist": 1.0},
            {"choose_every": 10},
            {"Kpid_x": [2.0, 0.0, 0.5]},
            {"Kpid_y": [2.0, 0.0, 0.5]},
            {"Kpid_theta": [0.3, 0.0, 1.0]},
            {"max_vel": [0.7, 0.7, 0.2]},
        ],
        remappings=[
            
        ],
    )

    return_home = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="return_home.py",
        parameters=[
            {"is_sim": False},
            {"red_left": False},
            # {"color_label_mappings_file": buoy_label_mappings},
            {"color_label_mappings_file": all_label_mappings},
            {"robot_frame_id": "base_link"},
            {"search_task_radius": 50.0},
            {"gate_dist_back": 5.0},
            {"gate_probe_dist": 10.0},
            {"gate_dist_thres": 50.0},
        ],
        remappings=[
        ]
    )

    follow_buoy_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_pid.py",
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": buoy_label_mappings},
            {"Kpid_x": [0.3, 0.0, 0.0]},
            {"Kpid_y": [0.3, 0.0, 0.0]},
            {"Kpid_theta": [0.01, 0.0, 0.0]},
            {"max_vel": [0.5, 0.5, 0.1]}
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
                "/mavros/imu/data/filtered"
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
            {"Kpid_x": [0.5, 0.0, 0.1]},
            {"Kpid_y": [0.5, 0.0, 0.1]},
            {"Kpid_theta": [0.6, 0.0, 0.0]},
            {"max_vel": [1.5, 1.0, 0.3]},
            {"avoid_max_dist": 5.0},
            {"avoid_vel_coeff": 1.0},
            {"rot_avoid_vel_coeff": 1.0},
            {"avoid_rot_vel_mag": True},
        ],
        output="screen",
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
            {"avoid_obs": True},
        ],
        output="screen",
    )

    navigation_server_tangent = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server_tangent.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "base_link"},
            {"Kpid_x": [0.5, 0.0, 0.1]},
            {"Kpid_y": [1.0, 0.0, 0.1]},
            {"Kpid_theta": [0.6, 0.0, 0.1]},
            {"max_vel": [1.5, 1.0, 0.3]},
            {"forward_dist": 4.0},
            {"avoid_obs": True},
            {"avoid_max_dist": 5.0},
            {"avoid_vel_coeff": 1.0},
            {"rot_avoid_vel_coeff": 1.0},
            {"avoid_rot_vel_mag": True},
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
        parameters=[
            {"is_sim": False},
            {"timer_period": 1.0},
        ],
    )

    delivery_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="delivery_server.py",
    )
    
    rviz_waypoint_sender = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="rviz_waypoint_sender.py",
        parameters=[
            {"xy_threshold": 3.0},
            {"theta_threshold": 180.0},
            {"use_waypoint_client": use_waypoint_client},
        ],
    )

    return [
        controller_server,
        # navigation_server,
        navigation_server_tangent,
        # navigation_server_nomap,
        rviz_waypoint_sender,
        run_tasks,
        task_init_server, 
        follow_buoy_path,
        speed_challenge,
        docking,
        mechanism_navigation,
        return_home,
        # follow_buoy_pid,
        # speed_challenge_pid
        # docking_fallback
        # delivery_server
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_waypoint_client", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
