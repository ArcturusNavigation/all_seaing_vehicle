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
                "back_right_port": 4,
                "back_left_port": 5,
            }
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
            {"range_radius": [0.5, 100000.0]},
        ],
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
            {"obstacle_size_min": 10},
            {"obstacle_size_max": 800},
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
            {"joy_x_scale": 2.0},
            {"joy_ang_scale": 0.8},
            {"serial_port": "/dev/ttyACM0"},
        ],
        condition=IfCondition(
            PythonExpression([
                "'", comms, "' == 'custom' and '", use_bag, "' == 'false'",
            ]),
        ),
    )

    yolov8_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="yolov8_node.py",
        output="screen",
        remappings=[
            ("image_raw", "/zed/zed_node/rgb/image_rect_color"),
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
            "port": "/dev/ttyACM1",
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
        condition=IfCondition(PythonExpression(["'", is_indoors, "' == 'true'"])),
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
        amcl_ld,
        lidar_ld,
        mavros_ld,
        static_transforms_ld,
        yolov8_node,
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
