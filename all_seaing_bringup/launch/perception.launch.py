from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml
import os
import yaml
import xacro

def launch_setup(context, *args, **kwargs):
    driver_prefix = get_package_share_directory("all_seaing_driver")
    description_prefix = get_package_share_directory("all_seaing_description")
    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    # set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    slam_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
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

    matching_weights = os.path.join(
        bringup_prefix, "config", "perception", "matching_weights.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )
    ransac_params = os.path.join(
        bringup_prefix, "config", "perception", "ransac_params.yaml"
    )

    buoy_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "best"},
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

    buoy_yolo_node_back_left = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "best"},
            {"label_config": "buoy_label_mappings"},
            {"conf": 0.6},
            {"use_color_names": False},
        ],
        remappings=[
            ("image", "/back_left_oak/rgb/image_rect"),
            ("annotated_image", "annotated_image/buoy/back_left"),
            ("bounding_boxes", "bounding_boxes/back_left"),
        ],
        output="screen",
    )

    buoy_yolo_node_back_right = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "best"},
            {"label_config": "buoy_label_mappings"},
            {"conf": 0.6},
            {"use_color_names": False},
        ],
        remappings=[
            ("image", "/back_right_oak/rgb/image_rect"),
            ("annotated_image", "annotated_image/buoy/back_right"),
            ("bounding_boxes", "bounding_boxes/back_right"),
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
            ("lidar_topic", "/point_cloud/filtered")
        ],
        parameters=[
            {"camera_name": "front"},
            {"base_link_frame": "base_link"},
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": inc_color_buoy_label_mappings},
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 1000},
            {"contour_bbox_area_thres": 0.5},
            {"cluster_bbox_area_thres": 0.0},
            {"clustering_distance": 0.1},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": False},
            {"label_list": True},
        ]
    )

    bbox_project_pcloud_node_back_left = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen",
        remappings=[
            ("camera_info_topic", "/back_left_oak/rgb/camera_info"),
            ("camera_topic", "/back_left_oak/rgb/image_rect"),
            ("lidar_topic", "/point_cloud/filtered"),
            ("bounding_boxes", "bounding_boxes/back_left"),
        ],
        parameters=[
            {"camera_name": "back_left"},
            {"base_link_frame": "base_link"},
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": inc_color_buoy_label_mappings},
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 1000},
            {"contour_bbox_area_thres": 0.5},
            {"cluster_bbox_area_thres": 0.0},
            {"clustering_distance": 0.1},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": False},
            {"label_list": True},
        ]
    )

    bbox_project_pcloud_node_back_right = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen",
        remappings=[
            ("camera_info_topic", "/back_right_oak/rgb/camera_info"),
            ("camera_topic", "/back_right_oak/rgb/image_rect"),
            ("lidar_topic", "/point_cloud/filtered"),
            ("bounding_boxes", "bounding_boxes/back_right"),
        ],
        parameters=[
            {"camera_name": "back_right"},
            {"base_link_frame": "base_link"},
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": inc_color_buoy_label_mappings},
            {"obstacle_size_min": 5},
            {"obstacle_size_max": 1000},
            {"contour_bbox_area_thres": 0.5},
            {"cluster_bbox_area_thres": 0.0},
            {"clustering_distance": 0.1},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": False},
            {"label_list": True},
        ]
    )

    multicam_detection_merge_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="multicam_detection_merge.py",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings = [
            ("detections/merged", "obstacle_map/local"),
        ],
        parameters = [
            {"enable_front": True},
            {"enable_back_left": True},
            {"enable_back_right": True},
            {"individual": False},
            {"approximate": False},
            {"delay": 0.1},
        ]
    )

    ransac_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="ransac_detector",
        output="screen",
        remappings=[
        ],
        parameters=[
            {"ransac_params_file": ransac_params},
            {"label_mappings_file": buoy_label_mappings},
            # {"label_mappings_file": shape_label_mappings},
        ]
    )

    point_cloud_filter_downsampled_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/velodyne_points"),
            ("point_cloud/filtered", "point_cloud/filtered_downsampled"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"range_radius": [1.0, 60.0]},
            {"leaf_size": 0.2},
            {"local_range_z": [-100000.0, 0.0]},
            {"min_pts_per_voxel": 4},
        ],
    )

    obstacle_detector_raw_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered_downsampled"),
        ],
        parameters=[
            {"base_link_frame": "base_link"},
            {"global_frame_id": "map"},
            {"clustering_distance": 0.2},
            {"obstacle_size_min": 4},
            {"range_max": 50.0},
        ],
    )

    obstacle_detector_unlabeled_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered_downsampled"),
            ("obstacle_map/raw", "obstacle_map/unlabeled")
        ],
        parameters=[
            {"base_link_frame": "base_link"},
            {"global_frame_id": "map"},
            {"clustering_distance": 0.4},
            {"obstacle_size_min": 4},
            # {"obstacle_size_max": 300},
            # {"obstacle_filter_pts_max": 100},
            # {"obstacle_filter_area_max": 0.2},
            {"obstacle_filter_length_max": 0.5},
            # {"range_max": 50.0},
        ],
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

    param_substitutions = {
        'track_robot': str(context.perform_substitution(LaunchConfiguration('use_slam')).lower() == "true"),
        'include_odom_only_theta': str((context.perform_substitution(LaunchConfiguration('use_gps')).lower() == "false") or 
                                       (context.perform_substitution(LaunchConfiguration('use_lio')).lower() == "true")),
    }

    configured_params = RewrittenYaml(
            source_file=slam_params,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("detections", "obstacle_map/local"),
            ("odometry/filtered", "odometry/gps"),
        ],
        parameters=[configured_params]
    )

    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms.launch.py",
            ]
        ),
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
        condition=UnlessCondition(LaunchConfiguration("use_slam")),
    )

    return [
        buoy_yolo_node,
        buoy_yolo_node_back_left,
        buoy_yolo_node_back_right,
        # shape_yolo_node,
        # static_shape_yolo_node,
        bbox_project_pcloud_node,
        bbox_project_pcloud_node_back_left,
        bbox_project_pcloud_node_back_right,
        multicam_detection_merge_node,
        # ransac_node,
        point_cloud_filter_downsampled_node,
        obstacle_detector_raw_node,
        obstacle_detector_unlabeled_node,
        grid_map_generator,
        object_tracking_map_node,
        # static_transforms_ld,
        map_to_odom,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            # DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument("location", default_value="pavillion"),
            DeclareLaunchArgument("use_slam", default_value='true'),
            DeclareLaunchArgument("use_gps", default_value='true'),
            DeclareLaunchArgument("use_lio", default_value='false'),
            OpaqueFunction(function=launch_setup),
        ]
    )
