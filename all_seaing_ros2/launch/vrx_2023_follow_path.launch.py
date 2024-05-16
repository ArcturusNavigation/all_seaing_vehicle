from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")

    robot_localization_params = os.path.join(
        get_package_share_directory("all_seaing_vehicle"),
        "params",
        "dual_ekf_navsat_sim.yaml",
    )

    return LaunchDescription(
        [
            # state reporter
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="nav_state_reporter",
                output="screen",
                remappings=[
                    ("/imu/data", "/wamv/sensors/imu/imu/data"),
                    ("/gps/fix", "/wamv/sensors/gps/gps/fix"),
                ],
            ),            
            # robot localization
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[robot_localization_params],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
                parameters=[robot_localization_params],
            ),
            # xdrive controller
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="xdrive_controller.py",
                name="controller",
                parameters=[{"in_sim": True}],
            ),
            # keyboard
            launch_ros.actions.Node(
                package="keyboard", executable="keyboard", name="keyboard"
            ),
            launch_ros.actions.Node(
                package="keyboard",
                executable="keyboard_to_joy.py",
                name="keyboard_to_joy",
                parameters=[
                    {
                        "config_file_name": os.path.join(
                            all_seaing_prefix, "params", "keyboard_config.yaml"
                        )
                    }
                ],
            ),
            # overlay node
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="cluster_bbox_overlay",
                output="screen",
                remappings=[
                    ("/img_info_src", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
                ],
            ),
            # color segmentation
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="color_segmentation.py",
                output="screen",
                remappings=[
                    ("/in_image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
                ],
            ),
            # pointcloud filter
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="pointcloud_filter",
                output="screen",
                remappings=[
                    ("/in_cloud", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
                    ("/out_cloud", "/filtered_cloud"),
                ],
                parameters=[
                    {"range_min_threshold": 0.0},
                    {"range_max_threshold": 40.0},
                    {"intensity_low_threshold": 0.0},
                    {"intensity_high_threshold": 50.0},
                    {"leaf_size": 0.0},
                    {"half_fov": 89}
                ],
            ),
            # pointcloud euclidean cluster detect
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="pointcloud_euclidean_cluster_detect",
                output="screen",
                remappings=[
                    ("/in_cloud", "/filtered_cloud"),
                ],
                parameters=[
                    {"cluster_size_min": 2},
                    {"cluster_size_max": 60},
                    {"clustering_distance": 1.0},
                    {"cluster_seg_thresh": 10.0},
                    {"drop_cluster_thresh": 1.0},
                    {"polygon_area_thresh": 100000.0},
                    {"viz": True},
                ],
            ),
            # buoy pair finder
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="buoy_pair_finder.py",
                output="screen",
            ),
            # follow the path
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [vrx_gz_prefix, "/launch/competition.launch.py"]
                ),
                launch_arguments={
                    "world": "practice_2023_follow_path2_task",
                    "urdf": f"{all_seaing_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
                }.items(),
            ),
           # MOOS-ROS bridge
            launch_ros.actions.Node(
                package="protobuf_client",
                executable="protobuf_client_node",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="moos_to_controller",
                name="moos_to_controller",
            ),
        ]
    )
