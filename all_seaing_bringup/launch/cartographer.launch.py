from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    cartographer_node = Node(
        package = "cartographer_ros",
        executable = "cartographer_node",
        arguments = [
            "-configuration_directory", FindPackageShare("all_seaing_bringup").find("all_seaing_bringup") + "/config/localization",
            "-configuration_basename", "cartographer.lua"],
        remappings = [
            ("echoes", "/scan"),
            ("points2", "/velodyne_points"),
            ("imu", "/zed/zed_node/imu/data")],
        output = "screen"
    )

    cartographer_occupancy_grid_node = Node(
        package = "cartographer_ros",
        executable = "cartographer_occupancy_grid_node",
        parameters = [{"resolution": 0.05}],
    )

    return LaunchDescription([
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
