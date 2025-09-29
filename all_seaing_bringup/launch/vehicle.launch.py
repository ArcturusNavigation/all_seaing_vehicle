from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.actions import SetRemap
import os
import yaml
import xacro
import numpy as np


def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    driver_prefix = get_package_share_directory("all_seaing_driver")
    perception_prefix = get_package_share_directory("all_seaing_perception")

    robot_urdf_file = os.path.join(
        description_prefix, "urdf", "fish_and_chips", "robot.urdf.xacro"
    )
    
    robot_localization_params = os.path.join(
        bringup_prefix, "config", "localization", "localize_real.yaml"
    )

    inc_color_buoy_label_mappings = os.path.join(
        bringup_prefix, "config", "perception", "inc_color_buoy_label_mappings.yaml"
    )
    slam_params = os.path.join(
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
    use_slam = context.perform_substitution(LaunchConfiguration("use_slam"))
    use_gps = context.perform_substitution(LaunchConfiguration("use_gps"))
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

    odometry_publisher_node = launch_ros.actions.Node(
        package = "all_seaing_driver",
        executable = "odometry_publisher.py",
        output = "screen",
        remappings=[
            ("gps_topic", "/mavros/global_position/raw/fix"),
            ("odom_topic", "/mavros/local_position/odom")
        ],
        parameters=[
            {"datum": [lat, lon, 0.0]},
            {"yaw_offset": np.pi/2.0},
            {"odom_yaw_offset": np.pi/2.0},
            {"utm_zone": 17 if location == "nbpark" else 19}, # 19 for Boston, 17 for Florida
        ]
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[
            {
                "global_frame_id": "map",
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
                "back_left_port": 5,
                "back_right_port":4,
            }
        ],
        condition=UnlessCondition(use_bag),
    )

    control_mux = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="control_mux.py",
    )

    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/velodyne_points"),
        ],
        parameters=[
            {"global_frame_id": "map"},
            {"range_radius": [0.5, 60.0]},
            {"leaf_size": 0.0},
        ],
        condition=UnlessCondition(use_bag),
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
            {"joy_ang_scale": 0.4},
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

    central_hub = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="central_hub_ros.py",
        parameters=[{"port": "/dev/ttyACM0"}],
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
            "port": "/dev/ttyACM1"
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
        launch_arguments={
            "indoors": is_indoors,
        }.items(),
        condition=UnlessCondition(use_bag),
    )

    robot_urdf = xacro.process_file(robot_urdf_file).toxml()
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
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

    perception_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                bringup_prefix,
                "/launch/perception.launch.py"
            ]
        ),
        launch_arguments={
            "location": location,
            "use_slam": use_slam,
            "use_gps": use_gps,
            "use_bag": context.perform_substitution(use_bag),
        }.items(),
    )
    
    tasks_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                bringup_prefix,
                "/launch/tasks.launch.py"
            ]
        ),
        launch_arguments={
            "location": location,
            "use_bag": context.perform_substitution(use_bag),
        }.items(),
    )

    return [
        control_mux,
        controller_node,
        # ekf_node,
        # navsat_node,
        odometry_publisher_node,
        point_cloud_filter_node,
        rover_custom_controller,
        rover_lora_controller,
        rviz_waypoint_sender,
        thrust_commander_node,
        central_hub,
        # amcl_ld,
        # static_transforms_ld,
        robot_state_publisher,
        # webcam_publisher,
        lidar_ld,
        mavros_ld,
        zed_ld,
        oak_ld,
        # perception_ld,
        # tasks_ld,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="boathouse"),
            DeclareLaunchArgument(
                "comms", default_value="custom", choices=["wifi", "lora", "custom"]
            ),
            DeclareLaunchArgument(
                "use_bag", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("use_slam", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("use_gps", default_value="true", choices=["true", "false"]),
            OpaqueFunction(function=launch_setup),
        ]
    )
