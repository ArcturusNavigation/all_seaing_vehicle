from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")
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

    obstacle_bbox_overlay_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_overlay",
        remappings=[
            (
                "camera_info",
                "/wamv/sensors/cameras/front_left_camera_sensor/camera_info",
            ),
        ],
    )

    color_segmentation_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="color_segmentation.py",
        remappings=[
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
    )

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
        ],
        parameters=[
            {"range_min_threshold": 0.0},
            {"range_max_threshold": 40.0},
            {"intensity_low_threshold": 0.0},
            {"intensity_high_threshold": 50.0},
            {"leaf_size": 0.0},
            {"hfov": 150.0},
        ],
    )

    obstacle_detector_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
        ],
        parameters=[
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 60},
            {"clustering_distance": 1.0},
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
            {"polygon_area_thresh": 100000.0},
        ],
    )

    buoy_pair_finder_node = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="buoy_pair_finder.py",
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
            "world": "practice_2023_follow_path2_task",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
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
            obstacle_bbox_overlay_node,
            color_segmentation_node,
            point_cloud_filter_node,
            obstacle_detector_node,
            buoy_pair_finder_node,
            protobuf_client_node,
            moos_to_controller_node,
            onshore_node,
            sim_ld,
        ]
    )
