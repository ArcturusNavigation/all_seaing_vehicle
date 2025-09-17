import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pc_share_dir = get_package_share_directory(
        "velodyne_pointcloud"
    )
    tf_params = {}
    tf_params["calibration"] = os.path.join(pc_share_dir, "params", "32db.yaml")
    tf_params["min_range"] = 0.9
    tf_params["max_range"] = 130.0
    tf_params["view_direction"] = 0.0
    tf_params["model"] = "32E"
    tf_params["organize_cloud"] = True
    transform_container = ComposableNodeContainer(
        name="velodyne_pointcloud_transform_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_pointcloud",
                plugin="velodyne_pointcloud::Transform",
                name="velodyne_convert_node",
                parameters=[tf_params],
            ),
        ],
        output="both",
    )

    driver_params = {}
    driver_params["device_ip"] = "192.168.1.201"
    driver_params["read_once"] = False
    driver_params["read_fast"] = False
    driver_params["repeat_delay"] = 0.0
    driver_params["frame_id"] = "velodyne"
    driver_params["model"] = "32E"
    driver_params["rpm"] = 1200.0 # change to change Hz (600 is 5Hz, 1200 is 10Hz and the maximum)
    driver_params["port"] = 2368
    driver_container = ComposableNodeContainer(
        name="velodyne_driver_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_driver",
                plugin="velodyne_driver::VelodyneDriver",
                name="velodyne_driver_node",
                parameters=[driver_params],
            ),
        ],
        output="both",
    )

    laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("velodyne_laserscan"),
                "/launch/velodyne_laserscan_node-composed-launch.py",
            ]
        ),
    )

    return LaunchDescription([driver_container, transform_container, laserscan_launch])
