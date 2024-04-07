from launch import LaunchDescription
import launch_ros

def generate_launch_description():

    return LaunchDescription(
        [
            # pointcloud filter
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="pointcloud_filter",
                output="screen",
                remappings=[
                    ("/in_cloud", "/velodyne_points"),
                    ("/out_cloud", "/filtered_cloud"),
                ],
                parameters=[
                    {"range_min_threshold": 0.0},
                    {"range_max_threshold": 100.0},
                    {"intensity_low_threshold": 0.0},
                    {"intensity_high_threshold": 100000.0},
                    {"leaf_size": 0.0},
                    {"half_fov": 89}
                ],
            ),
            # pointcloud euclidean cluster detect
            launch_ros.actions.Node(
                package="all_seaing_vehicle",
                executable="pointcloud_euclidean_cluster_detect",
                output="screen",
                remappings=[
                    ("/in_cloud", "/filtered_cloud"),
                ],
                parameters=[
                    {"cluster_size_min": 20},
                    {"cluster_size_max": 100000},
                    {"clustering_distance": 0.2},
                    {"cluster_seg_thresh": 5.0},
                    {"drop_cluster_thresh": 1.0},
                    {"polygon_area_thresh": 100000.0},
                    {"viz": True},
                ],
            ),
        ]
    )
