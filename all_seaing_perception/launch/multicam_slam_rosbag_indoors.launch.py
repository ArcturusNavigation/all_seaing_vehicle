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
import numpy as np

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
    ransac_params = os.path.join(
        bringup_prefix, "config", "perception", "ransac_params.yaml"
    )
    contour_matching_color_ranges = os.path.join(
        bringup_prefix, "config", "perception", "contour_matching_color_ranges.yaml"
    )

    set_use_sim_time = launch_ros.actions.SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )

    robot_localization_rf2o_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_rf2o.yaml"
    )

    robot_localization_amcl_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_amcl.yaml"
    )

    imu_filter_params = os.path.join(
        driver_prefix, "config", "imu_filter.yaml"
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
            # {"model": "best"},
            {"model": "roboboat_shape_2025"},
            # {"label_config": "buoy_label_mappings"},
            {"label_config": "shape_label_mappings"},
            {"conf": 0.1},
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
    
    ransac_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="ransac_detector",
        output="screen",
        remappings=[
            # ("labeled_object_point_clouds", "labeled_object_point_clouds/merged"),
            ("labeled_object_point_clouds", "labeled_object_point_clouds/front"),
        ],
        parameters=[
            {"ransac_params_file": ransac_params},
            # {"label_mappings_file": buoy_label_mappings},
            {"label_mappings_file": shape_label_mappings},
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
            # ("detections", "obstacle_map/local"),
            ("detections", "detections/front"),
            # ("odometry/filtered", "odometry/gps"),
            # ("odometry/filtered", "odometry/integrated"),
            ("odometry/filtered", "odom_rf2o/filtered")
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
            # {"child_frames_to_remove": ["imu_link_accel"]},
            # {"parent_frames_to_remove": ["map"]},
            {"parent_frames_to_remove": ["map", "odom", "odom_rf2o"]},
        ]
    )

    robot_urdf = xacro.process_file(robot_urdf_file).toxml()
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    odometry_publisher_node = launch_ros.actions.Node(
        package = "all_seaing_driver",
        executable = "odometry_publisher.py",
        output = "screen",
        remappings=[
            ("gps_topic", "/mavros/global_position/raw/fix"),
            ("odom_topic", "/mavros/local_position/odom"),
            # ("pos_odom_topic", "/mavros/local_position/odom"),
        ],
        parameters=[
            {"datum": [42.358541, -71.087389, 0.0]},
            # {"yaw_offset": -np.pi/2.0},
            # {"odom_yaw_offset": -np.pi/2.0},
            {"yaw_offset": np.pi/2.0},
            {"odom_yaw_offset": np.pi/2.0},
            {"utm_zone": 19}, # 19 for Boston, 17 for Florida
            # {"use_odom_pos": True},
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

    point_cloud_filter_downsampled_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/point_cloud/filtered"),
            ("point_cloud/filtered", "point_cloud/filtered_downsampled"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"range_radius": [0.5, 60.0]},
            {"leaf_size_xy": 0.3},
            {"leaf_size_z": 0.3},
            {"local_range_z": [-100000.0, 0.0]},
            {"min_pts_per_voxel": 10},
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
            {"clustering_distance": 0.25},
            {"obstacle_size_min": 3},
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
            {"clustering_distance": 1.0},
            {"obstacle_size_min": 2},
            # {"obstacle_size_max": 300},
            # {"obstacle_filter_pts_max": 100},
            # {"obstacle_filter_area_max": 0.2},
            {"obstacle_filter_length_max": 0.3},
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

    rotate_imu_accel = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--qx",
            "1.0",
            "--qy",
            "0.0",
            "--qz",
            "0.0",
            "--qw",
            "0.0",
            "--frame-id",
            "imu_link_odom",
            "--child-frame-id",
            "imu_link_accel",
        ],
    )

    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[imu_filter_params],
        remappings=[
            ("imu/data_raw", "/mavros/imu/data"),
            ("imu/mag", "/mavros/imu/mag"),
            ("imu/data", "/mavros/imu/data/filtered"),
        ]
    )

    imu_reframe_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="imu_reframe.py",
        parameters=[
            {"target_frame_id": "imu_link_accel"},
            {"zero_g": True},
            {"flip_gyro": True},
        ],
        remappings=[
            ("imu_topic", "/mavros/imu/data/filtered"),
            ("new_imu_topic", "/mavros/imu/data/reframed")
        ]
    )

    pcl_to_scan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/point_cloud/filtered'),
                    ('scan', '/pcl_scan')],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.5,
            'max_height': 5.0,
            'angle_min': -np.pi,
            'angle_max': np.pi,
            'angle_increment': np.pi/360.0,
            'scan_time': 1/30.0,
            'range_min': 3.0,
            'range_max': 60.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
    )

    rf2o_node = launch_ros.actions.Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/pcl_scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom_rf2o2',
            'init_pose_from_topic' : '',
            'freq' : 20.0}],
    )
    
    ekf_node_rf2o = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_rf2o_params],
        remappings=[
            ("odometry/filtered", "odom_rf2o/filtered"),
        ],
    )

    ekf_node_amcl = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_amcl_params],
        remappings=[
            ("odometry/filtered", "odometry/integrated"),
        ],
    )


    amcl_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                bringup_prefix,
                "/launch/amcl.launch.py"
            ]
        ),
        launch_arguments={
            "location": "z_center",
        }.items(),
    )

    return [
        set_use_sim_time,
        # ekf_node,
        buoy_yolo_node,
        # buoy_yolo_node_back_left,
        # buoy_yolo_node_back_right,
        bbox_project_pcloud_node,
        # bbox_project_pcloud_node_back_left,
        # bbox_project_pcloud_node_back_right,
        ransac_node,
        # multicam_detection_merge_node,
        # odometry_publisher_node,
        object_tracking_map_node,
        tf_filtering,
        # robot_state_publisher,
        # static_transforms_ld,
        # point_cloud_filter_node,
        # point_cloud_filter_downsampled_node,
        # obstacle_detector_raw_node,
        # obstacle_detector_unlabeled_node,
        # grid_map_generator,
        # rotate_imu_accel,
        imu_reframe_node,
        imu_filter_node,
        pcl_to_scan_node,
        rf2o_node,
        ekf_node_rf2o,
        # ekf_node_amcl,
        amcl_ld,
    ]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            OpaqueFunction(function=launch_setup),
        ]
    )
