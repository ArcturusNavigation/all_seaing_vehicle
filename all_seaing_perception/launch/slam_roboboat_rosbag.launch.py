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
import numpy as np

def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    description_prefix = get_package_share_directory("all_seaing_description")
    utility_prefix = get_package_share_directory("all_seaing_utility")
    odom_transformer_params = os.path.join(
        description_prefix, "config", "odom_transformer_params.yaml"
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
    slam_rosbag_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_rosbag.yaml"
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

    odometry_publisher_node = launch_ros.actions.Node(
        package = "all_seaing_driver",
        executable = "odometry_publisher.py",
        output = "screen",
        remappings=[
            ("gps_topic", "/mavros/global_position/global"),
            ("odom_topic", "/odometry/filtered"),
            ("pos_odom_topic", "/odometry/gps_fake"),
        ],
        parameters=[
            {"base_link_frame": "actual_base_link"},
            {"datum": [27.3729, -82.4537, np.pi/2.0]},
            # {"magnetic_declination": 0.14},
            {"odom_yaw_offset": -np.pi/2.0},
            {"use_odom_pos": True},
        ]
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
            {"parent_frames_to_remove": ["base_link", "map", "odom"]},
        ]
    )

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/point_cloud/filtered_fake"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"range_radius": [0.5, 60.0]},
        ]
    )

    obstacle_detector_raw_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
        ],
        parameters=[
            {"base_link_frame": "actual_base_link"},
            {"global_frame_id": "map"},
            {"clustering_distance": 1.0},
            {"obstacle_size_min": 5},
            {"range_max": 50.0},
        ],
    )

    obstacle_detector_unlabeled_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
            ("obstacle_map/raw", "obstacle_map/unlabeled")
        ],
        parameters=[
            {"base_link_frame": "actual_base_link"},
            {"global_frame_id": "map"},
            {"clustering_distance": 1.0},
            {"obstacle_size_min": 5},
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
            ("odometry/filtered", "odometry/tracked"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"timer_period": 1.0},
            {"grid_dim": [1000, 1000]},
            {"grid_resolution": 0.1},
            {"obstacle_radius_sigma": 3.0},
            {"search_radius_sigma": 10.0},
            {"dynamic_origin": True},
        ],
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
            # {"base_link_frame": "base_link"},
            {"bbox_object_margin": 10.0},
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
            # ("odometry/filtered", "odometry_correct/filtered")
            ("odometry/filtered", "odometry/gps"),
        ],
        parameters=[slam_rosbag_params]
    )

    return [
        odometry_publisher_node,
        static_transforms_ld,
        tf_filtering,
        point_cloud_filter_node,
        obstacle_detector_raw_node,
        obstacle_detector_unlabeled_node,
        # grid_map_generator,
        bbox_project_pcloud_node,
        object_tracking_map_node,
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
