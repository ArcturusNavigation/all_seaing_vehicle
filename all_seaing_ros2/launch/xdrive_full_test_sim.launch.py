from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# this will run everything except the heartbeat, so you can directly Ctrl+C the heartbeat


def generate_launch_description():
    vrx_gz_prefix = get_package_share_directory("vrx_gz")
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")
    bag_path = os.path.join(os.path.dirname(os.path.dirname(get_package_prefix("all_seaing_vehicle"))), "src", "all_seaing_vehicle", "all_seaing_ros2", "bag")

    robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat_sim.yaml")
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
        # state reporter
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="nav_state_reporter",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")
            ]
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="protobuf_client",
            executable="protobuf_client_node",
            output="screen"),
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="xdrive_controller.py",
            name="controller",
            parameters=[{"in_sim": True}]
        ),
        # task_manager_mode,
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="rviz_testing_helper.py",
            name="rviz_testing_helper",
            parameters=[{"bag_path": bag_path}]
        ),
        launch_ros.actions.Node(
            package="keyboard",
            executable="keyboard",
            name="keyboard"
        ),
        launch_ros.actions.Node(
            package="keyboard",
            executable="keyboard_to_joy.py",
            name="keyboard_to_joy",
            parameters=[{"config_file_name": os.path.join(all_seaing_prefix, "params", "keyboard_config.yaml")}]
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(all_seaing_prefix, "params", "rviz_config.rviz")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{all_seaing_prefix}/urdf/xdrive_wamv/wamv_target.urdf"}.items()),
    ])
