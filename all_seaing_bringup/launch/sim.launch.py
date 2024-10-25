from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch_ros
import os
import subprocess


def generate_launch_description():

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_sim.yaml"
    )
    color_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "color_label_mappings.yaml"
    )
    color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "color_ranges.yaml"
    )

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")

    launch_rviz_launch_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"]
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/wamv/sensors/gps/gps/fix")],
        parameters=[robot_localization_params],
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        remappings=[
            ("thrusters/front_left/thrust", "/wamv/thrusters/front_left/thrust"),
            ("thrusters/front_right/thrust", "/wamv/thrusters/front_right/thrust"),
            ("thrusters/back_left/thrust", "/wamv/thrusters/back_left/thrust"),
            ("thrusters/back_right/thrust", "/wamv/thrusters/back_right/thrust"),
        ],
        parameters=[
            {
                "front_right_xy": [1.1, -1.0],
                "back_left_xy": [-2.4, 1.0],
                "front_left_xy": [1.1, 1.0],
                "back_right_xy": [-2.4, -1.0],
                "thruster_angle": 45.0,
                "drag_constants": [5.0, 5.0, 40.0],
                "output_range": [-1500.0, 1500.0],
                "smoothing_factor": 0.8,
            }
        ],
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

    perception_eval_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="perception_eval.py",
    )

    obstacle_bbox_visualizer_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_visualizer",
        remappings=[
            ("camera_info", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
            }
        ],
    )

    color_segmentation_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="color_segmentation.py",
        remappings=[
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
                "color_ranges_file": color_ranges,
            }
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

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": True},
        ],
        arguments=[
            "-d",
            os.path.join(bringup_prefix, "rviz", "dashboard.rviz"),
        ],
        condition=IfCondition(launch_rviz),
    )

    control_mux = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="control_mux.py",
    )

    controller_server = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="controller_server.py",
        parameters=[
            {"global_frame_id": "odom"},
            {"Kpid_x": [1.0, 0.0, 0.0]},
            {"Kpid_y": [1.0, 0.0, 0.0]},
            {"Kpid_theta": [1.0, 0.0, 0.0]},
            {"max_vel": [5.0, 3.0, 1.5]},
        ],
        output="screen",
    )

    onshore_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="onshore_node.py",
        output="screen",
        parameters=[
            {"joy_x_scale": 5.0},
            {"joy_y_scale": -3.0},
            {"joy_ang_scale": -1.5},
        ],
    )


    waypoint_finder = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="waypoint_finder.py",
        parameters=[
            {"color_label_mappings_file": color_label_mappings},
            {"safe_margin": 0.2}
        ]
    )

    rviz_waypoint_sender = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="rviz_waypoint_sender.py",
        parameters=[
            {"xy_threshold": 1.0},
            {"theta_threshold": 5.0},
        ],
        output="screen",
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
        launch_arguments={
            "world": "follow_path_task",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
            "extra_gz_args": "-v 0",
        }.items(),
    )

    return LaunchDescription(
        [
            launch_rviz_launch_arg,
            ekf_node,
            navsat_node,
            controller_node,
            obstacle_bbox_overlay_node,
            obstacle_bbox_visualizer_node,
            color_segmentation_node,
            point_cloud_filter_node,
            obstacle_detector_node,
            rviz_node,
            control_mux,
            controller_server,
            onshore_node,
            waypoint_finder,
            rviz_waypoint_sender,
            keyboard_ld,
            sim_ld,
            perception_eval_node,
        ]
    )
