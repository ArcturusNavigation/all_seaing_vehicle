from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# sample launch file to run the sydney regatta sim with an xdrive boat and controller


def generate_launch_description():
    vrx_gz_prefix = get_package_share_directory("vrx_gz")
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")

    robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat_sim.yaml")
    controller_node = launch_ros.actions.Node(
        package="all_seaing_vehicle",
        executable="xdrive_controller.py",
        name="controller",
        parameters=[{"in_sim": True}],
        condition=IfCondition(LaunchConfiguration("with_control")),
        on_exit=[
            EmitEvent(event=Shutdown(reason="controller died"))
        ]
    )
    return LaunchDescription([
        DeclareLaunchArgument("with_control", default_value=TextSubstitution(text="True")),
        # launch_ros.actions.Node(
	    #     package="all_seaing_vehicle",
        #     executable="state_reporter",
        #     remappings=[
        #         ("/imu/data", "/wamv/sensors/imu/imu/data"),
        #         ("/gps/fix", "/wamv/sensors/gps/gps/fix")
        #     ]
        # ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[robot_localization_params]),
        # MOOS-ROS bridge
        launch_ros.actions.Node(
            package="protobuf_client",
            executable="protobuf_client_node",
            output="screen",
        ),

        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
            parameters=[robot_localization_params]),
        controller_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{all_seaing_prefix}/urdf/xdrive_wamv/wamv_target.urdf"}.items()),
    ])
