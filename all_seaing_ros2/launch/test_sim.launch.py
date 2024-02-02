from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os


def generate_launch_description():
    nav2_prefix = get_package_share_directory("all_seaing_vehicle")
    vrx_gz_prefix = get_package_share_directory("vrx_gz")

    robot_localization_params = os.path.join(
        get_package_share_directory("all_seaing_vehicle"),
        "params",
        "dual_ekf_navsat_sim.yaml",
    )

    return LaunchDescription(
        [
            # controller
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="simple_controller.py",
                output="screen",
                remappings=[
                    ("/left_thrust", "/wamv/thrusters/left/thrust"),
                    ("/right_thrust", "/wamv/thrusters/right/thrust"),
                ],
                parameters=[
                    {"linear_scaling": 25.0},
                    {"angular_scaling": 15.0},
                    {"lower_thrust_limit": -1400.0},
                    {"upper_thrust_limit": 1400.0},
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

            # rviz
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-f", "odom"]
            ),
            
            # static map generation
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="static_map_generator.py",
                output="screen"
            ),
            # overlay node
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="pointcloud_image_overlay",
                output="screen",
                remappings=[
                    (
                        "/img_src",
                        "/wamv/sensors/cameras/front_left_camera_sensor/image_raw",
                    ),
                    (
                        "/img_info_src",
                        "/wamv/sensors/cameras/front_left_camera_sensor/camera_info",
                    ),
                    ("/cloud_src", "/wamv/sensors/lidars/lidar_wamv_sensor/points"),
                ],
            ),
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
            # waypoint sender
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="waypoint_sender.py",
                output="screen",
                parameters=[{"use_pose_array": True}, {"use_gps": False}],
            ),
            # obstacle sender
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="obstacle_sender.py",
                output="screen",
                parameters=[{"use_gps": False}],
            ),
            # buoy pair finder
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="buoy_pair_finder.py",
                output="screen",
            ),
           IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_prefix, "/launch/nav2.launch.py"]
                )
            ),
            # default simulation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [vrx_gz_prefix, "/launch/competition.launch.py"]
                )
            ),
            # MOOS-ROS bridge
            launch_ros.actions.Node(
                package="protobuf_client",
                executable="protobuf_client_node",
                output="screen",
            ),
        ]
    )
