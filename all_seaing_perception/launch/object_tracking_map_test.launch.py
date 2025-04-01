from launch import LaunchDescription
import launch_ros

def generate_launch_description():
    object_tracking_map_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="object_tracking_map",
        output="screen",
        remappings=[
            ("camera_info_topic", "/wamv/sensors/cameras/front_left_camera_sensor/camera_info"),
        ],
        parameters=[
            {"obstacle_drop_thresh": 2.0},
            {"range_uncertainty": 1.0},
            {"bearing_uncertainty": 0.1},
            {"new_object_slam_threshold": 5.0},
            {"init_new_cov": 10.0},
            {"track_robot": True},
            {"check_fov": True},
            {"is_sim": True},
        ]
    )
    return LaunchDescription([
        object_tracking_map_node
    ])
