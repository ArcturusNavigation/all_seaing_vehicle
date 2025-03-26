from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
import os
import yaml


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
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
    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )

    with open(locations_file, "r") as f:
        locations = yaml.safe_load(f)

    location = context.perform_substitution(LaunchConfiguration("location"))
    comms = LaunchConfiguration("comms")
    use_bag = LaunchConfiguration("use_bag")
    is_indoors = str(locations[location]["indoors"]).lower()

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
        condition=IfCondition(
            PythonExpression([
                "'", is_indoors, "' == 'false' and '", use_bag, "' == 'false'"
            ]),
        ),
    )
        
    lat = locations[location]["lat"]
    lon = locations[location]["lon"]
    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        parameters=[
            robot_localization_params,
            {"datum": [lat, lon, 0.0]},
        ],
        condition=IfCondition(
            PythonExpression([
                "'", is_indoors, "' == 'false' and '", use_bag, "' == 'false'",
            ]),
        ),
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[
            {
                "front_right_xy": [0.27, -0.27],
                "back_left_xy": [-0.27, 0.27],
                "front_left_xy": [0.27, 0.27],
                "back_right_xy": [-0.27, -0.27],
                "thruster_angle": 67.5,
                "drag_constants": [50.0, 50.0, 200.0],
                "output_range": [1100.0, 1900.0],
                "smoothing_factor": 0.8,
            }
        ],
    )

    thrust_commander_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="thrust_commander.py",
        parameters=[
            {
                "front_right_port": 2,
                "front_left_port": 3,
                "back_left_port": 4,
                "back_right_port": 5,
            }
        ],
        condition=UnlessCondition(use_bag),
    )

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
    )

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_path.py",
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": color_label_mappings},
            {"safe_margin": 0.2},
        ],
    )

    follow_buoy_pid = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_pid.py",
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": color_label_mappings},
            {"forward_speed": 1.2},
            {"max_yaw": 0.2},
            {"pid_vals": [0.0009, 0.0, 0.0003]},
        ],
        remappings=[
            ("camera_info", "/zed/zed_node/rgb/camera_info"),
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
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"Kpid_x": [0.3, 0.0, 0.0]},
            {"Kpid_y": [0.3, 0.0, 0.0]},
            {"Kpid_theta": [0.3, 0.0, 0.0]},
            {"max_vel": [1.5, 1.0, 0.3]},
        ],
        output="screen",
    )

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/velodyne_points"),
        ],
        parameters=[
            {"range_radius": [0.5, 60.0]},
        ],
        condition=UnlessCondition(use_bag),
    )

    obstacle_bbox_overlay_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_overlay",
        remappings=[
            (
                "camera_info",
                "/zed/zed_node/rgb/camera_info",
            ),
        ],
    )

    obstacle_detector_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
        ],
        parameters=[
            {"obstacle_size_min": 5},
            {"obstacle_size_max": 300},
            {"clustering_distance": 0.1},
        ],
    )

    obstacle_bbox_visualizer_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_visualizer",
        remappings=[
            (
                "camera_info",
                "/zed/zed_node/rgb/camera_info",
            ),
            ("image", "/zed/zed_node/rgb/image_rect_color"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
            }
        ],
    )

    rviz_waypoint_sender = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="rviz_waypoint_sender.py",
        parameters=[
            {"xy_threshold": 2.0},
            {"theta_threshold": 180.0},
        ],
        condition=UnlessCondition(use_bag),
        output="screen",
    )

    rover_lora_controller = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="rover_lora_controller.py",
        condition=IfCondition(
            PythonExpression([
                "'", comms, "' == 'lora' and '", use_bag, "' == 'false'",
            ]),
        ),
        output="screen",
    )

    rover_custom_controller = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="rover_custom_controller.py",
        parameters=[
            {"joy_x_scale": -1.8},
            {"joy_ang_scale": 0.2},
        ],
        condition=IfCondition(
            PythonExpression([
                "'", comms, "' == 'custom' and '", use_bag, "' == 'false'",
            ]),
        ),
    )

    webcam_publisher = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="webcam_publisher.py",
        parameters=[
            {"video_index": 0},
        ],
        remappings=[
            ("webcam_image", "turret_image"),
        ]
    )

    grid_map_generator = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="grid_map_generator.py",
        parameters=[
            {"timer_period": 1.0},
            {"grid_dim": [800, 800]},
            {"default_range": 60},
            {"grid_resolution": 0.1},
        ],
    )

    buoy_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_2025"},
            {"label_config": "color_label_mappings"},
            {"conf": 0.6},
            {"use_color_names": True},
        ],
        remappings=[
            ("image", "/zed/zed_node/rgb/image_rect_color"),
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
            {"use_color_names": False},
        ],
        remappings=[
            ("image", "turret_image"),
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
            ("image", "/zed/zed_node/rgb/image_rect_color"),
            ("annotated_image", "annotated_image/shape"),
            ("bounding_boxes", "static_shape_boxes"),
        ],
        output="screen",
    )

    bbox_project_pcloud_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen",
        remappings=[
            ("camera_info_topic", "/zed/zed_node/rgb/camera_info"),
            ("camera_topic", "/zed/zed_node/rgb/image_rect_color"),
            ("lidar_topic", "/point_cloud/filtered"),
            ("bounding_boxes", "static_shape_boxes")
        ],
        parameters=[
            {"bbox_object_margin": 1.0},
            {"color_label_mappings_file": buoy_label_mappings},
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 60},
            {"clustering_distance": 1.0},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": False},
            {"label_list": True},
        ]
    )

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/zed/zed_node/rgb/camera_info"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"slam_frame_id": "slam_map"},
            {"obstacle_drop_thresh": 2.0},
            {"normalize_drop_thresh": False},
            {"range_uncertainty": 20.0},
            {"bearing_uncertainty": 0.2},
            {"motion_gps_xy_noise": 1.0},
            {"motion_gps_theta_noise": 0.1},
            {"motion_imu_xy_noise": 1.0},
            {"motion_imu_theta_noise": 0.05},
            {"update_gps_xy_uncertainty": 5000.0},
            {"new_object_slam_threshold": 2.0},
            {"init_new_cov": 10.0},
            {"check_fov": False},
            {"track_robot": True},
            {"imu_predict": True},
            {"gps_update": True},
            {"direct_tf": False},
            {"is_sim": False},
        ]
    )

    object_tracking_map_euclidean_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map_euclidean",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/zed/zed_node/rgb/camera_info"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
            {"check_fov": False},
            {"is_sim": True},
        ]
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"robot_frame_id": "wamv/wamv/base_link"},
        ],
        output="screen",
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        parameters=[{"is_sim": False}],
    )

    delivery_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="delivery_server.py",
    )

    central_hub = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="central_hub_ros.py",
        parameters=[{"port": "/dev/ttyACM2"}],
    )

    lidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/32e_points.launch.py",
            ]
        ),
        condition=UnlessCondition(use_bag),
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
        condition=UnlessCondition(use_bag),
    )

    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/zed2i.launch.py",
            ]
        ),
        condition=UnlessCondition(use_bag),
    )

    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms.launch.py",
            ]
        ),
        launch_arguments={
            "indoors": is_indoors,
        }.items(),
        condition=UnlessCondition(use_bag),
    )

    amcl_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                bringup_prefix,
                "/launch/amcl.launch.py"
            ]
        ),
        launch_arguments={
            "location": location,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", is_indoors, "' == 'true' and '", use_bag, "' == 'false'",
            ]),
        ),
    )
    
    return [
        control_mux,
        controller_node,
        controller_server,
        ekf_node,
        navsat_node,
        navigation_server,
        obstacle_bbox_overlay_node,
        obstacle_bbox_visualizer_node,
        obstacle_detector_node,
        point_cloud_filter_node,
        rover_custom_controller,
        rover_lora_controller,
        rviz_waypoint_sender,
        thrust_commander_node,
        buoy_yolo_node,
        # shape_yolo_node,
        # static_shape_yolo_node,
        bbox_project_pcloud_node,
        object_tracking_map_node,
        run_tasks,
        task_init_server, 
        follow_buoy_path,
        follow_buoy_pid,
        grid_map_generator,
        central_hub,
        amcl_ld,
        static_transforms_ld,
        webcam_publisher,
        lidar_ld,
        mavros_ld,
        zed_ld,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            DeclareLaunchArgument(
                "comms", default_value="wifi", choices=["wifi", "lora", "custom"]
            ),
            DeclareLaunchArgument(
                "use_bag", default_value="false", choices=["true", "false"]
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
