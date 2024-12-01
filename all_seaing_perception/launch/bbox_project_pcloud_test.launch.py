from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():
    bbox_project_pcloud_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="bbox_project_pcloud",
        output="screen"
    )
    return LaunchDescription([
        bbox_project_pcloud_node
    ])