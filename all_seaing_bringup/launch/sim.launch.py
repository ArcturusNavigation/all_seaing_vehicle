from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import launch_ros
import os
import subprocess
import yaml


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_sim.yaml"
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
    color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "color_ranges.yaml"
    )
    ycrcb_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "color_ranges_LED_ycrcb.yaml"
    )
    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )
    buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
    )

    subprocess.run(["cp", "-r", os.path.join(bringup_prefix, "tile"), "/tmp"])

    launch_rviz = LaunchConfiguration("launch_rviz")
    no_gui = str(context.perform_substitution(LaunchConfiguration("no_gui")))
    use_waypoint_client = LaunchConfiguration("use_waypoint_client")

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    with open(locations_file, "r") as f:
        locations = yaml.safe_load(f)
    lat = locations["sydney"]["lat"]
    lon = locations["sydney"]["lon"]
    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/wamv/sensors/gps/gps/fix")],
        parameters=[
            robot_localization_params,
            {"datum": [lat, lon, 0.0]},
        ],
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
        parameters=[{"is_sim": True}],
    )

    perception_eval_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="perception_eval.py",
    )

    obstacle_bbox_visualizer_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_visualizer",
        remappings=[
            (
                "camera_info",
                "/wamv/sensors/cameras/front_left_camera_sensor/camera_info",
            ),
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
            },
            {
                "is_sim": True,
            },
        ],
    )

    ycrcb_color_segmentation_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="color_segmentation_ycrcb.py",
        remappings=[
            ("/webcam_image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
                "color_ranges_file": ycrcb_color_ranges,
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

    yolov8_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        output="screen",
        remappings=[
            ("image_raw", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
        ],
        parameters=[
            {
                "use_color_names": True,
            }
        ]
    )

    buoy_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_2025"},
            {"label_config": "color_label_mappings"},
            {"conf": 0.6},
        ],
        remappings=[
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
            ("annotated_image", "annotated_image/buoy"),
        ],
        output="screen",
    )

    shape_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_shape_2025"},
            {"label_config": "shape_label_mappings"},
            {"conf": 0.4},
        ],
        remappings=[
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
            ("annotated_image", "annotated_image/shape"),
            ("bounding_boxes", "shape_boxes"),
        ],
        output="screen",
    )

    static_shape_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_shape_2025"},
            {"label_config": "shape_label_mappings"},
            {"conf": 0.4},
        ],
        remappings=[
            ("image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
            ("annotated_image", "annotated_image/shape"),
            ("bounding_boxes", "static_shape_boxes"),
        ],
        output="screen",
    )

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
        ],
        parameters=[
            {"range_x": [0.0, 100000.0]},
            {"range_y": [5.0, 100000.0]},
            {"range_radius": [1.0, 100000.0]},
            {"range_intensity": [0.0, 50.0]},
            {"leaf_size": 0.0},
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
        ],
    )

    bbox_project_pcloud_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen",
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
            ("camera_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
            ("lidar_topic", "point_cloud/filtered")
        ],
        parameters=[
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": color_buoy_label_mappings},
            {"color_ranges_file": color_ranges},
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 60},
            {"clustering_distance": 1.0},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": True},
            {"label_list": True}
        ]
    )

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"slam_frame_id": "slam_map"},
            {"obstacle_drop_thresh": 2.0},
            {"normalize_drop_thresh": False},
            {"range_uncertainty": 10.0},
            {"bearing_uncertainty": 0.1},
            {"motion_gps_xy_noise": 1.0},
            {"motion_gps_theta_noise": 0.1},
            {"motion_imu_xy_noise": 10.0},
            {"motion_imu_theta_noise": 0.01},
            {"update_gps_xy_uncertainty": 5.0},
            {"new_object_slam_threshold": 2.0},
            {"init_new_cov": 10.0},
            {"check_fov": False},
            {"track_robot": True},
            {"imu_predict": True},
            {"gps_update": True},
            {"direct_tf": False},
            {"is_sim": True},
        ]
    )

    object_tracking_map_euclidean_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map_euclidean",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
            {"check_fov": False},
            {"is_sim": True},
        ]
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
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"Kpid_x": [1.0, 0.0, 0.0]},
            {"Kpid_y": [1.0, 0.0, 0.0]},
            {"Kpid_theta": [1.0, 0.0, 0.0]},
            {"max_vel": [5.0, 3.0, 1.5]},
        ],
        output="screen",
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"robot_frame_id": "wamv/wamv/base_link"},
        ],
        output="screen",
    )

    grid_map_generator = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="grid_map_generator.py",
        remappings=[("scan", "/wamv/sensors/lidars/lidar_wamv_sensor/scan")],
        parameters=[
            {"timer_period": 1.0},
            {"grid_dim": [800, 800]},
            {"grid_resolution": 0.3},
            {"obstacle_radius_sigma": 3.0},
            {"search_radius_sigma": 15.0}
        ],
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

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_path.py",
        parameters=[
            {"is_sim": True},
            {"color_label_mappings_file": color_label_mappings},
            {"safe_margin": 0.2},
        ],
    )

    speed_challenge_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="speed_challenge_pid.py",
        parameters=[
            {"is_sim": True},
            {"color_label_mappings_file": color_label_mappings},
        ],
        remappings=[
            (
                "camera_info",
                "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"
            ),
            (
                "imu",
                "/wamv/sensors/imu/imu/data"
            )
        ],
    )

    follow_buoy_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_pid.py",
        parameters=[
            {"is_sim": True},
            {"color_label_mappings_file": color_label_mappings},
        ],
        remappings=[
            (
                "camera_info",
                "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"
            )
        ],
    )

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
        # parameters=[{"is_sim": True}],
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        parameters=[{"is_sim": True}],
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

    map_to_odom = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
    )

    if no_gui == "true":
        extra_gz_args = "-v -s 0"
    else:
        extra_gz_args = "-v 0"
    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
        launch_arguments={
            # "world": "rb2025/rb2025_task1_task2.sdf",
            "world": "follow_path_task.sdf",
            # "world": "speed_course_world.sdf",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
            "extra_gz_args": extra_gz_args,
        }.items(),
    )

    return [
        ekf_node,
        navsat_node,
        controller_node,
        controller_server,
        obstacle_bbox_overlay_node,
        obstacle_bbox_visualizer_node,
        obstacle_detector_node,
        color_segmentation_node,
        ycrcb_color_segmentation_node,
        # yolov8_node,
        point_cloud_filter_node,
        bbox_project_pcloud_node,
        object_tracking_map_node,
        # object_tracking_map_euclidean_node,
        rviz_node,
        control_mux,
        navigation_server,
        grid_map_generator,
        onshore_node,
        run_tasks,
        task_init_server,
        # follow_buoy_path,
        follow_buoy_pid,
        speed_challenge_pid,
        rviz_waypoint_sender,
        map_to_odom,
        keyboard_ld,
        sim_ld,
        perception_eval_node,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "no_gui", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "use_waypoint_client", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
