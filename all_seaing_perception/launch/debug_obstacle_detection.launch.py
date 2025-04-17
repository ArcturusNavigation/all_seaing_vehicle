from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
import os
import yaml

def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    description_prefix = get_package_share_directory("all_seaing_description")
    utility_prefix = get_package_share_directory("all_seaing_utility")
    color_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "color_label_mappings.yaml"
    )
    color_buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "color_buoy_label_mappings.yaml"
    )
    inc_color_buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "inc_color_buoy_label_mappings.yaml"
    )
    buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
    )
    shape_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "shape_label_mappings.yaml"
    )
    color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "color_ranges.yaml"
    )
    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )
    slam_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
    )
    pf_slam_params = os.path.join(
        bringup_prefix, "config", "slam", "pf_slam_real.yaml"
    )
    locations_file = os.path.join(
        bringup_prefix, "config", "localization", "locations.yaml"
    )

    with open(locations_file, "r") as f:
        locations = yaml.safe_load(f)

    location = context.perform_substitution(LaunchConfiguration("location"))
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
    
    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms_rosbag.launch.py",
            ]
        ),
        launch_arguments={
            "indoors": is_indoors,
        }.items(),
    )

    tf_filtering = launch_ros.actions.Node(
        package="all_seaing_utility",
        executable="filter_tf.py",
        parameters=[
            {"old_tf_topic": "/tf_fake"},
            {"new_tf_topic": "/tf"},
            {"old_static_tf_topic": "/tf_static_fake"},
            {"new_static_tf_topic": "/tf_static"},
            # {"child_frames_to_remove": ["zed_camera_link"]},
            {"parent_frames_to_remove": ["base_link"]},
        ]
    )

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        parameters=[{"is_sim": True}],
    )

    keyboard_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([driver_prefix, "/launch/keyboard.launch.py"]),
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
        # condition=UnlessCondition(use_bag),
    )
    
    color_segmentation_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="color_segmentation.py",
        remappings=[
            ("image", "/zed/zed_node/rgb/image_rect_color"),
        ],
        parameters=[
            {
                "color_label_mappings_file": color_label_mappings,
                "color_ranges_file": color_ranges,
            }
        ],
    )

    obstacle_detector_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
        ],
        parameters=[
            {"obstacle_size_min": 10},
            {"obstacle_size_max": 800},
            {"clustering_distance": 0.1},
        ],
    )

    buoy_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_2025"},
            {"label_config": "buoy_label_mappings"},
            {"conf": 0.6},
            {"use_color_names": False},
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
            {"use_color_names": False},
        ],
        remappings=[
            ("image", "/zed/zed_node/rgb/image_rect_color"),
            ("annotated_image", "annotated_image/static_shape"),
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
            ("lidar_topic", "/point_cloud/filtered")
        ],
        parameters=[
            {"base_link_frame": "actual_base_link"},
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": inc_color_buoy_label_mappings},
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
        parameters=[slam_params]
    )

    object_tracking_map_pf_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map_pf",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("camera_info_topic", "/zed/zed_node/rgb/camera_info"),
        ],
        parameters=[pf_slam_params],
    )

    object_tracking_map_euclidean_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map_euclidean",
        output="screen",
        remappings=[
            ("camera_info_topic", "/zed/zed_node/rgb/camera_info"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"obstacle_seg_thresh": 10.0},
            {"obstacle_drop_thresh": 1.0},
            {"check_fov": True},
            {"is_sim": False},
        ]
    )

    obstacle_bbox_overlay_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_bbox_overlay",
        remappings=[
            ("camera_info", "/zed/zed_node/rgb/camera_info"),
            # ("obstacle_map/raw", "obstacle_map/refined_untracked")
        ],
        parameters=[
        ]
    )

    follow_buoy_path = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="follow_buoy_path.py",
        remappings=[
            ("obstacle_map/labeled", "obstacle_map/refined_tracked"),
        ],
        parameters=[
            {"is_sim": False},
            {"color_label_mappings_file": color_label_mappings},
            {"safe_margin": 0.2},
        ],
    )

    return [
        # set_use_sim_time,
        ekf_node,
        navsat_node,
        keyboard_ld,
        static_transforms_ld,
        tf_filtering,
        # run_tasks,
        # task_init_server, 
        # follow_buoy_path,
        buoy_yolo_node,
        # shape_yolo_node,
        # static_shape_yolo_node,
        bbox_project_pcloud_node,
        object_tracking_map_node,
        # object_tracking_map_pf_node,
        # object_tracking_map_euclidean_node,
        # obstacle_detector_node,
        # color_segmentation_node,
        # obstacle_bbox_overlay_node
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            DeclareLaunchArgument(
                "comms", default_value="wifi", choices=["wifi", "lora", "custom"]
            ),
            DeclareLaunchArgument(
                "use_bag", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
