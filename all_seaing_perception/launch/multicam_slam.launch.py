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

    slam_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
    )
    locations_file = os.path.join(
        bringup_prefix, "config", "localization", "locations.yaml"
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params]
    )

    with open(locations_file, "r") as f:
        locations = yaml.safe_load(f)

    location = context.perform_substitution(LaunchConfiguration("location"))

    lat = locations[location]["lat"]
    lon = locations[location]["lon"]
    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        parameters=[
            robot_localization_params,
            # {"datum": [lat, lon, 0.0]},
        ],remappings=[
            ("/imu/data", "/mavros/imu/data")
        ]
    )

    odometry_publisher_node = launch_ros.actions.Node(
        package = "all_seaing_driver",
        executable = "odometry_publisher.py",
        output = "screen",
        remappings=[
            ("gps_topic", "/mavros/global_position/raw/fix"),
            ("odom_topic", "/mavros/local_position/odom")
        ],
        parameters=[

        ]
    )

    oak_ld = GroupAction(
        actions=[
            SetRemap(src='/tf',dst='/tf_trash'),
            SetRemap(src='/tf_static',dst='/tf_static_trash'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        driver_prefix,
                        "/launch/oak.launch.py",
                    ]
                ),
            )
        ]
    )
    
    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms.launch.py",
            ]
        ),
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

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/velodyne_points"),
        ],
        parameters=[
            {"range_radius": [0.5, 60.0]},
        ]
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

    lidar_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/32e_points.launch.py",
            ]
        ),
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
    )

    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                driver_prefix,
                "/launch/zed2i.launch.py",
            ]
        ),
    )

    robot_urdf = xacro.process_file(robot_urdf_file).toxml()
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    static_transforms_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                description_prefix,
                "/launch/static_transforms.launch.py",
            ]
        )
    )

    return [
        mavros_ld,
        # ekf_node,
        # navsat_node,
        odometry_publisher_node,
        oak_ld,
        buoy_yolo_node,
        buoy_yolo_node_back_left,
        buoy_yolo_node_back_right,
        point_cloud_filter_node,
        bbox_project_pcloud_node,
        bbox_project_pcloud_node_back_left,
        bbox_project_pcloud_node_back_right,
        # multicam_detection_merge_node,
        # object_tracking_map_node,
        lidar_ld,
        zed_ld,
        robot_state_publisher,
        # static_transforms_ld
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
