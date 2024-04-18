from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# this will run everything needed to run on the actual boat 

def generate_launch_description():
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")

    robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat.yaml")
    task_manager_mode = launch_ros.actions.Node(
        package="all_seaing_vehicle",
        executable="task_manager.py",
        name="task_manager",
        condition=IfCondition(LaunchConfiguration("run_tasks")),
        on_exit=[
            EmitEvent(event=Shutdown(reason="task manager lost heartbeat"))
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument("run_tasks", default_value=TextSubstitution(text="True")),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            remappings=[("/gps/fix", "/mavros/global_position/raw/fix")],
            parameters=[robot_localization_params]),
        #launch_ros.actions.Node(
        #    package="all_seaing_vehicle",
        #    executable="xdrive_controller.py",
        #    name="controller",
        #    parameters=[{"in_sim": False}]
        #),
        task_manager_mode
    ])
