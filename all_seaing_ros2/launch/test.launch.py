# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch.conditions import IfCondition
# import launch_ros
# import os

# # sample launch file to run the sydney regatta sim with an xdrive boat and controller

# def generate_launch_description():
#     all_seaing_prefix = get_package_share_directory("all_seaing_vehicle") 
#     robot_localization_params = os.path.join(all_seaing_prefix, "params", "dual_ekf_navsat_sim.yaml")
#     return LaunchDescription([
#         launch_ros.actions.Node(
#             package="robot_localization",
#             executable="navsat_transform_node",
#             name="navsat_transform_node",
#             output="screen",
#             remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
#             parameters=[robot_localization_params])
#     ])
