import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/path/to/your/map.yaml', description='Path to the map file'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/arcturus/dev_ws/src/all_seaing_vehicle/all_seaing_bringup/map/sea_grant.yaml',
            }]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'min_particles': 1000,
                'max_particles': 5000,
                'global_frame_id': 'map',
                'base_frame_id': 'zed_camera_link',
            }],
        ),
    ])

