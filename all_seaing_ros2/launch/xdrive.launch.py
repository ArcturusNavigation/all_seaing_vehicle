from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# sample launch file to run the sydney regatta sim with an xdrive boat and controller


def generate_launch_description():
    all_seaing_prefix = get_package_share_directory("all_seaing_vehicle")

    robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("with_control", default_value=TextSubstitution(text="True")),
        # launch_ros.actions.Node(
	    #     package="all_seaing_vehicle", 
        #     executable="nav_state_reporter",
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
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="xdrive_controller.py",
            name="controller",
            parameters=[{"in_sim": False}],
            condition=IfCondition(LaunchConfiguration("with_control"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{all_seaing_prefix}/urdf/xdrive_wamv/wamv_target.urdf"}.items()),
    ])
