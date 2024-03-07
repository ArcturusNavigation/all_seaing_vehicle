import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # velodyne pointcloud convert node
    tf_params = {}
    tf_params["calibration"] = os.path.join(
        get_package_share_directory(
            "velodyne_pointcloud"
        ), 
        "params",
        "32db.yaml"
    )
    tf_params["min_range"] = 0.9
    tf_params["max_range"] = 130.0
    tf_params["view_direction"] = 0.0
    tf_params["organize_cloud"] = True
    transform_container = ComposableNodeContainer(
        name="velodyne_pointcloud_transform_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="velodyne_pointcloud",
                plugin="velodyne_pointcloud::Convert",
                name="velodyne_convert_node",
                parameters=[tf_params],
            ),
        ],
        output="both",
    )

    # velodyne laserscan
    laserscan_node = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="screen",
    )

    # velodyne driver
    driver_params = {}
    driver_params["device_ip"] = "192.168.1.201"
    driver_params["read_once"] = False
    driver_params["read_fast"] = False
    driver_params["repeat_delay"] = 0.0
    driver_params["frame_id"] = "velodyne"
    driver_params["model"] = "32E"
    driver_params["rpm"] = 600.0
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

    # mavros
    mavros_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(
                "all_seaing_vehicle"
            ),
            "/launch/mavros.launch.py"
        ])
    )

    # zed
    zed_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(
                "zed_wrapper"
            ),
            "/launch/obsolete/zed2i.launch.py"
        ])
    )

    # pwm sender
    pwm_sender_node = Node(
        package="all_seaing_vehicle"
    )

    laserscan_node = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        output="screen",
    )


    return LaunchDescription([
        driver_container, 
        transform_container,
        laserscan_node,
        mavros_ld,
        zed_ld
    ])
