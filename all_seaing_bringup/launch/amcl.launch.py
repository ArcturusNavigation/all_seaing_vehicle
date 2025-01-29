from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
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
        }],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
        }]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        output="screen",
        parameters=[{
            "min_particles": 500,
            "max_particles": 2000,
            "global_frame_id": "map",
            "base_frame_id": "zed_camera_link",
        }],
    )

    return [
        map_server,
        amcl_node,
        lifecycle_manager,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("location", default_value="seagrant"),
            OpaqueFunction(function=launch_setup),
        ]
    )

