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

        # overlay node
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="pointcloud_image_overlay",
            output="screen",
            remappings=[
                ("/img_src", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
                ("/img_info_src", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
                ("/cloud_src", "/wamv/sensors/lidars/lidar_wamv_sensor/points")]),

        # state reporter
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="nav_state_reporter", 
            output="screen",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")]),

        # default simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"])),
       
        # MOOS-ROS bridge
        launch_ros.actions.Node(
	        package="protobuf_client", executable="protobuf_client_node", output="screen"),
    ])

