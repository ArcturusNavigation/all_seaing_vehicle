from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import launch_ros

def generate_launch_description():

    vrx_gz_prefix = get_package_share_directory("vrx_gz") 

    return LaunchDescription([
        # controller
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="simple_controller.py",
            output="screen",
            remappings=[
                ("/left_thrust", "/wamv/thrusters/left/thrust"),
                ("/right_thrust", "/wamv/thrusters/right/thrust")],
            parameters=[
                {"linear_scaling": 25.0},
                {"angular_scaling": 15.0},
                {"lower_thrust_limit": -1400.0},
                {"upper_thrust_limit": 1400.0}]),

        # state reporter
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="nav_state_reporter", 
            output="screen",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")]),

        # wayfinding
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "acoustic_perception_task"}.items()),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="vrx_2023_acoustic_sub.py",
            output="screen"),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="vrx_2023_acoustic_sender.py",
            output="screen"),
       
        # MOOS-ROS bridge
        launch_ros.actions.Node(
	        package="protobuf_client", executable="protobuf_client_node", output="screen"),
    ])

