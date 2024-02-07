import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("all_seaing_vehicle"),
        "params",
        "nav2_params_sim.yaml",
    )

    lifecycle_nodes = ["smoother_server", "planner_server"]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    load_node = Node(
        name="nav2_container",
        package="rclcpp_components",
        executable="component_container_isolated",
        parameters=[params_file, {"autostart": "true"}],
        remappings=remappings,
        output="screen",
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="nav2_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_smoother",
                plugin="nav2_smoother::SmootherServer",
                name="smoother_server",
                parameters=[params_file],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[params_file],
                remappings=remappings,
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(load_node)
    ld.add_action(load_composable_nodes)
    return ld
