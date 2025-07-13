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
import os
import yaml
import xacro

def launch_setup(context, *args, **kwargs):
    driver_prefix = get_package_share_directory("all_seaing_driver")
    description_prefix = get_package_share_directory("all_seaing_description")
    bringup_prefix = get_package_share_directory("all_seaing_bringup")

    robot_urdf_file = os.path.join(
        description_prefix, "urdf", "fish_and_chips", "robot.urdf.xacro"
    )
    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )
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

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params]
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

    buoy_yolo_node_back_left = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "roboboat_2025"},
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
            {"model": "roboboat_2025"},
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
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 1000},
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

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("detections", "detections/merged"),
        ],
        parameters=[slam_params]
    )

    tf_filtering = launch_ros.actions.Node(
        package="all_seaing_utility",
        executable="filter_tf.py",
        parameters=[
            {"old_tf_topic": "/tf_fake"},
            {"new_tf_topic": "/tf"},
            {"old_static_tf_topic": "/tf_static_fake"},
            {"new_static_tf_topic": "/tf_static"},
            {"child_frames_to_remove": ["slam_map"]},
            # {"parent_frames_to_remove": ["velodyne"]},
        ]
    )

    robot_urdf = xacro.process_file(robot_urdf_file).toxml()
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    return [
        # ekf_node,
        # buoy_yolo_node,
        # buoy_yolo_node_back_left,
        # buoy_yolo_node_back_right,
        # bbox_project_pcloud_node,
        # bbox_project_pcloud_node_back_left,
        # bbox_project_pcloud_node_back_right,
        # multicam_detection_merge_node,
        object_tracking_map_node,
        tf_filtering,
        # robot_state_publisher,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
