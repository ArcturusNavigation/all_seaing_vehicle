from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():

    vrx_gz_prefix = get_package_share_directory("vrx_gz") 

    return LaunchDescription([
       # state reporter
        launch_ros.actions.Node(
	        package="all_seaing_vehicle", 
            executable="nav_state_reporter", 
            output="screen",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")]),

        # object detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "perception_task"}.items()),
        launch_ros.actions.Node(
	        package="all_seaing_vehicle",
            executable="yolov5_detector.py",
            output="screen",
            remappings=[
                ("/perception_suite/image", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw")]),

        # overlay node
        launch_ros.actions.Node(
            package="all_seaing_vehicle",
            executable="pointcloud_image_overlay",
            output="screen",
            remappings=[
                ("/img_src", "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"),
                ("/img_info_src", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
                ("/cloud_src", "/wamv/sensors/lidars/lidar_wamv_sensor/points")]),
    ])

