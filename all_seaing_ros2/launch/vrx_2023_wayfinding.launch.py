from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
import launch_ros
import os


def generate_launch_description():

    vrx_gz_prefix = get_package_share_directory("vrx_gz")
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")
    robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat_sim.yaml")

    return LaunchDescription(
        [
            # controller
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="xdrive_controller.py",
                name="controller",
                parameters=[{"in_sim": True}],
                on_exit=[
                EmitEvent(event=Shutdown(reason="task manager lost heartbeat"))
        ]
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
            # localization
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[robot_localization_params]),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                parameters=[robot_localization_params]),
            # waypoint sender
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="waypoint_sender.py",
                output="screen",
                remappings=[("/waypoints", "/vrx/wayfinding/waypoints")],
                parameters=[{"use_pose_array": True}, {"use_gps": True}],
            ),
            # wayfinding
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [vrx_gz_prefix, "/launch/competition.launch.py"]
                ),
                launch_arguments={"world": "wayfinding_task", "urdf": f"{all_seaing_prefix}/urdf/xdrive_wamv/wamv_target.urdf"}.items(),
            ),
            #Keyboard/Teleop
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

            # MOOS-ROS bridge
            launch_ros.actions.Node(
                package="protobuf_client",
                executable="protobuf_client_node",
                output="screen",
            ),

            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="moos_to_controller",
                name = "moos_to_controller"
            )
        ]
    )
