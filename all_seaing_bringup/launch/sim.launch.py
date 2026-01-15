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
    task_locations = os.path.join(
        bringup_prefix, "config", "course", "task_locations_sim.yaml"
    )
    slam_real_params = os.path.join(
        bringup_prefix, "config", "slam", "slam_real.yaml"
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
    shape_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "shape_label_mappings.yaml"
    )
    ransac_params = os.path.join(
        bringup_prefix, "config", "perception", "ransac_params.yaml"
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
    utm = locations["sydney"]["utm"]
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

    buoy_yolo_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        parameters=[
            {"model": "best"},
            # {"model": "roboboat_shape_2025"},
            {"label_config": "buoy_label_mappings"},
            # {"label_config": "shape_label_mappings"},
            # {"use_color_names": False},
            {"conf": 0.3},
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
            {"global_frame_id": "map"},
            {"range_x": [0.0, 100000.0]},
            {"range_y": [5.0, 100000.0]},
            {"range_radius": [1.0, 100000.0]},
            {"range_intensity": [0.0, 50.0]},
        ],
    )

    point_cloud_filter_obstacle_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
            ("point_cloud/filtered", "point_cloud/filtered_obs")
        ],
        parameters=[
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"global_frame_id": "map"},
            {"range_radius": [1.0, 15.0]},
            {"range_intensity": [0.0, 50.0]},
            {"local_range_z": [-100000.0, 0.0]},
            {"leaf_size_xy": 3.0},
            {"leaf_size_z": 3.0},
            {"min_pts_per_voxel": 10},
            {"convert_to_robot": True},
        ],
    )

    obstacle_detector_raw_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="obstacle_detector",
        remappings=[
            ("point_cloud", "point_cloud/filtered"),
        ],
        parameters=[
            {"base_link_frame": "wamv/wamv/base_link"},
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
            {"base_link_frame": "wamv/wamv/base_link"},
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

    bbox_project_pcloud_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen",
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
            ("camera_topic", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
            ("lidar_topic", "point_cloud/filtered"),
            ("detections", "obstacle_map/local"),
        ],
        parameters=[
            {"base_link_frame": "wamv/wamv/base_link"},
            {"bbox_object_margin": 0.0},
            {"color_label_mappings_file": color_buoy_label_mappings},
            {"color_ranges_file": color_ranges},
            {"obstacle_size_min": 2},
            {"obstacle_size_max": 2000},
            {"contour_bbox_area_thres": 0.5},
            {"cluster_bbox_area_thres": 0.0},
            {"clustering_distance": 0.1},
            {"matching_weights_file": matching_weights},
            {"contour_matching_color_ranges_file": contour_matching_color_ranges},
            {"is_sim": True},
            {"label_list": True}
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
            # {"label_mappings_file": buoy_label_mappings},
            {"label_mappings_file": shape_label_mappings},
        ]
    )

    odometry_publisher_node = launch_ros.actions.Node(
        package = "all_seaing_driver",
        executable = "odometry_publisher.py",
        output = "screen",
        remappings=[
            ("gps_topic", "/wamv/sensors/gps/gps/fix"),
            ("odom_topic", "odometry/filtered"),
            ("odometry/gps", "odometry/gps_sim"),
        ],
        parameters=[
            {"datum": [lat, lon, 0.0]},
            # {"yaw_offset": np.pi/2.0},
            # {"odom_yaw_offset": np.pi/2.0},
            {"yaw_offset": 0.0},
            {"odom_yaw_offset": 0.0},
            {"utm_zone": utm}, # 19 for Boston, 17 for Florida
            {"publish_tf": False},
            {"base_link_frame": "wamv/wamv/base_link"},
            {"use_sim_time": True},
        ]
    )

    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ("detections", "obstacle_map/local"),
            # ("odometry/filtered", "odometry/gps_sim"),
            ("odometry/filtered", "odometry/integrated"),
        ],
        parameters=[slam_real_params],
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
            {"global_frame_id": "map"},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"Kpid_x": [1.0, 0.0, 0.0]},
            {"Kpid_y": [1.0, 0.0, 0.0]},
            {"Kpid_theta": [1.0, 0.0, 0.0]},
            {"max_vel": [5.0, 3.0, 1.5]},
            {"avoid_max_dist": 5.0},
            {"avoid_vel_coeff": 3.0},
        ],
        output="screen",
    )

    navigation_server = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"avoid_obs": True},
        ],
        output="screen",
    )

    navigation_server_nomap = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server_nomap.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "wamv/wamv/base_link"},
        ],
        output="screen",
    )

    navigation_server_tangent = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="navigation_server_tangent.py",
        parameters=[
            {"global_frame_id": "map"},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"forward_dist": 15.0},
            {"avoid_obs": True},
        ],
        output="screen",
    )

    grid_map_generator = launch_ros.actions.Node(
        package="all_seaing_navigation",
        executable="grid_map_generator.py",
        parameters=[
            {"global_frame_id": "map"},
            {"timer_period": 1.0},
            {"grid_dim": [800, 800]},
            {"grid_resolution": 0.3},
            {"obstacle_radius_sigma": 3.0},
            {"search_radius_sigma": 15.0}
        ],
        remappings=[
            ("scan", "/wamv/sensors/lidars/lidar_wamv_sensor/scan"),
            ("odometry/filtered", "odometry/tracked"),
        ]
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
            {"robot_frame_id": "wamv/wamv/base_link"},
            # {"midpoint_pair_forward_dist": 5.0},
        ],
        remappings=[
            ("odometry/filtered", "odometry/tracked"),
        ]
    )

    speed_challenge = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="speed_challenge.py",
        parameters=[
            {"is_sim": True},
            {"color_label_mappings_file": color_label_mappings},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"probe_distance": 30},
            # {"init_gate_dist": 5.0},
            # {"gate_dist_back": 5.0},
        ],
        remappings=[
            ("odometry/filtered", "odometry/tracked"),
        ]
    )

    docking = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="docking.py",
        parameters=[
            {"is_sim": True},
            # {"shape_label_mappings_file": buoy_label_mappings},
            {"shape_label_mappings_file": shape_label_mappings},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"dock_width": 2.0},
            {"dock_length": 11.0},
            {"wpt_banner_dist": 12.0},
            {"navigation_dist_thres": 15.0},
            {"duplicate_dist": 0.3},
            {"docked_xy_thres": 1.0},
            {"Kpid_x": [0.75, 0.0, 0.0]},
            {"Kpid_y": [0.75, 0.0, 0.0]},
            {"Kpid_theta": [0.75, 0.0, 0.0]},
            {"max_vel": [2.0, 1.0, 0.4]},
            {"avoid_max_dist": 5.0},
            {"avoid_vel_coeff": 3.0},
        ],
        remappings=[
            
        ],
    )
    
    mechanism_navigation = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="mechanism_navigation.py",
        parameters=[
            {"is_sim": True},
            # {"shape_label_mappings_file": buoy_label_mappings},
            {"shape_label_mappings_file": shape_label_mappings},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"wpt_banner_dist": 10.0},
            {"navigation_dist_thres": 25.0},
            # {"update_target_dist_thres": 3.0},
            {"shooting_xy_thres": 1.0},
            {"shooting_theta_thres": 10.0},
            {"duplicate_dist": 0.7},
            {"choose_every": 10},
            {"Kpid_x": [0.75, 0.0, 0.0]},
            {"Kpid_y": [0.75, 0.0, 0.0]},
            {"Kpid_theta": [0.75, 0.0, 0.0]},
            {"max_vel": [1.0, 1.0, 0.3]},
        ],
        remappings=[
            
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
            ),
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
            ),
        ],
    )

    docking_fallback = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="docking_fallback.py",
        parameters=[
            {"is_sim": True},
            # {"shape_label_mappings_file": buoy_label_mappings},
            {"shape_label_mappings_file": shape_label_mappings},
            {"robot_frame_id": "wamv/wamv/base_link"},
            {"dock_width": 2.0},
            {"dock_length": 11.0},
        ],
        remappings=[
            
        ],
    )

    run_tasks = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="run_tasks.py",
        parameters=[
            {"is_sim": True},
            {"color_label_mappings_file": color_label_mappings},
            {"task_locations_file": task_locations},
            {"robot_frame_id": "wamv/wamv/base_link"},
        ]
    )

    task_init_server = launch_ros.actions.Node(
        package="all_seaing_autonomy",
        executable="task_init.py",
        parameters=[
            {"is_sim": True},
            {"timer_period": 1.0},
        ],
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

    extra_gz_args = "-v -s 0" if no_gui == "true" else "-v 0"
    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
        launch_arguments={
            # "world": "rb2025/rb2025_task1_task2.sdf",
            "world": "rb2025/follow_path_new.sdf",
            # "world": "follow_path_task.sdf",
            # "world": "speed_course_world.sdf",
            # "world": "scan_dock_deliver_task.sdf",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
            "extra_gz_args": extra_gz_args,
        }.items(),
    )

    return [
        ekf_node,
        navsat_node,
        controller_node,
        controller_server,
        obstacle_detector_raw_node,
        obstacle_detector_unlabeled_node,
        color_segmentation_node,
        # ycrcb_color_segmentation_node,
        # buoy_yolo_node,
        point_cloud_filter_node,
        point_cloud_filter_obstacle_node,
        bbox_project_pcloud_node,
        ransac_node,
        odometry_publisher_node,
        object_tracking_map_node,
        rviz_node,
        control_mux,
        # navigation_server,
        # navigation_server_nomap,
        navigation_server_tangent,
        grid_map_generator,
        onshore_node,
        run_tasks,
        task_init_server,
        follow_buoy_path,
        speed_challenge,
        docking,
        mechanism_navigation,
        # follow_buoy_pid,
        # speed_challenge_pid,
        # docking_fallback,
        rviz_waypoint_sender,
        # map_to_odom,
        keyboard_ld,
        sim_ld,
        # perception_eval_node,
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
