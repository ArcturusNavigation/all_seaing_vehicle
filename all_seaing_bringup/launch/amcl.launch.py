from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
import os

def launch_setup(context, *args, **kwargs):

    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    location = context.perform_substitution(LaunchConfiguration("location"))

    map_file = os.path.join(
        bringup_prefix, "map", f"{location}.yaml"
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{
            "yaml_filename": map_file,
            "frame_id": "odom",
        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        parameters=[{
            "min_particles": 1000,
            "max_particles": 2000,
            "global_frame_id": "odom",
            "base_frame_id": "base_link",
            "scan_topic": "pcl_scan",
            "odom_frame_id": "odom_rf2o",
            # "tf_broadcast": False,
        }],
        remappings=[
            # ("scan", "pcl_scan"),
        ]
    )

    return [
        lifecycle_manager,
        map_server,
        amcl_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="z_center"),
            OpaqueFunction(function=launch_setup),
        ]
    )

