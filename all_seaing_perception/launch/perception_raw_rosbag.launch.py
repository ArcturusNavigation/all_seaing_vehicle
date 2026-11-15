"""
Replay a rosbag through the real perception pipeline from raw sensor data, reproducing
on-boat behavior and real-time performance (issues #326, #480, #501).

Unlike the other *_rosbag launch files (which replay processed topics such as
/point_cloud/filtered to debug SLAM/tasks), this one recomputes everything after the drivers.

Only raw sensor data is played from the bag: the LiDAR, the three cameras and their
camera_info, MAVROS (IMU, odometry, GPS) and /tf_static (the sensor extrinsics recorded
with the bag). Everything else is computed live, exactly as on the boat:
perception.launch.py (YOLO, fusion, merging, obstacle detection, grid map, SLAM/tracking)
plus the processing nodes that perception relies on from vehicle.launch.py:
point_cloud_filter (/point_cloud/filtered, the cloud used for LiDAR-camera fusion) and
odometry_publisher (odom -> base_link). A Foxglove bridge (ws://<jetson>:8765) is started too. Recorded outputs such as /tf, detections and
maps are not played, so they can't mix with the live ones.

Run it on the Jetson to measure real performance. Compressed camera topics must be
decoded beforehand with all_seaing_utility/rosbag_decompress_images.py, so the replay
doesn't pay for decompression the boat doesn't do.

Usage:
    ros2 launch all_seaing_perception perception_raw_rosbag.launch.py bag:=<bag_dir>
        [location:=pavillion] [rate:=1.0] [start_offset:=0.0] [play_delay:=20.0] [loop:=false]
        [<any perception.launch.py argument, e.g. use_slam:=true>]

Logs of the run go to $ARCTURUS_LOG_DIR/replay_<bag name>_<timestamp>/ (see run_logger.py).
Stop with Ctrl+C or `ros2 run all_seaing_utility kill_ros_nodes.sh`.
"""

import datetime
import os

import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Raw sensor topics replayed from the bag (everything else is recomputed live)
RAW_TOPICS = [
    "/velodyne_points",
    "/zed/zed_node/rgb/image_rect_color",
    "/zed/zed_node/rgb/camera_info",
    "/back_left_oak/rgb/image_rect",
    "/back_left_oak/rgb/camera_info",
    "/back_right_oak/rgb/image_rect",
    "/back_right_oak/rgb/camera_info",
    "/mavros/imu/data",
    "/mavros/imu/data_raw",
    "/mavros/imu/mag",
    "/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/global_position/raw/fix",
    "/mavros/global_position/global",
    "/tf_static",
]

# perception.launch.py arguments forwarded unchanged
PERCEPTION_ARGS = {
    "use_slam": "false",
    "use_gps": "true",
    "track_banners": "true",
    "banners_slam": "true",
    "use_lio": "false",
    "use_amcl": "false",
}


def launch_setup(context, *args, **kwargs):
    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    bag = os.path.abspath(os.path.expanduser(LaunchConfiguration("bag").perform(context)))
    location = LaunchConfiguration("location").perform(context)

    with open(os.path.join(bringup_prefix, "config", "localization", "locations.yaml")) as f:
        loc = yaml.safe_load(f)[location]

    # One run-log folder per replay, named after the bag
    run_id = "replay_{}_{}".format(os.path.basename(bag.rstrip("/")),
                                   datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

    perception_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_prefix, "/launch/perception.launch.py"]),
        launch_arguments={
            "use_sim_time": "true",
            "location": location,
            **{k: LaunchConfiguration(k).perform(context) for k in PERCEPTION_ARGS},
        }.items(),
    )

    # Same configuration as odometry_publisher_node in vehicle.launch.py (keep in sync)
    odometry_publisher_node = launch_ros.actions.Node(
        package="all_seaing_driver",
        executable="odometry_publisher.py",
        output="screen",
        remappings=[
            ("gps_topic", "/mavros/global_position/raw/fix"),
            ("odom_topic", "/mavros/local_position/odom"),
        ],
        parameters=[
            {"use_sim_time": True},
            {"datum": [loc["lat"], loc["lon"], 0.0]},
            {"yaw_offset": 0.0},
            {"odom_yaw_offset": 0.0},
            {"utm_zone": loc["utm"]},
        ],
    )

    # Same configuration as point_cloud_filter_node in vehicle.launch.py (keep in sync)
    point_cloud_filter_node = launch_ros.actions.Node(
        package="all_seaing_perception",
        executable="point_cloud_filter",
        remappings=[
            ("point_cloud", "/velodyne_points"),
        ],
        parameters=[
            {"use_sim_time": True},
            {"global_frame_id": "map"},
            {"range_radius": [0.0, 60.0]},
            {"filter_bbox": [-0.5, -1.0, -1.0, 0.5, 0.5, 0.5]},  # (min_x, min_y, min_z, max_x, max_y, max_z)
        ],
    )

    play_cmd = ["ros2", "bag", "play", bag, "--clock",
                "--rate", LaunchConfiguration("rate").perform(context),
                "--start-offset", LaunchConfiguration("start_offset").perform(context),
                "--topics", *RAW_TOPICS]
    if LaunchConfiguration("loop").perform(context).lower() == "true":
        play_cmd.append("--loop")
    # Wait for the nodes (YOLO model loading takes a while) before playing, so the start of
    # the bag isn't lost
    bag_play = TimerAction(
        period=float(LaunchConfiguration("play_delay").perform(context)),
        actions=[ExecuteProcess(cmd=play_cmd, output="screen")],
    )

    # Foxglove bridge (ws://<jetson>:8765), as in vehicle.launch.py, to visualize the replay from the laptop
    foxglove_ld = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory("foxglove_bridge"), "launch",
                                                "foxglove_bridge_launch.xml")),
        launch_arguments={"port": "8765", "use_sim_time": "true"}.items(),
    )

    return [
        SetEnvironmentVariable("ARCTURUS_RUN_ID", run_id),
        foxglove_ld,
        perception_ld,
        point_cloud_filter_node,
        odometry_publisher_node,
        bag_play,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", description="bag directory to replay"),
            DeclareLaunchArgument("location", default_value="pavillion"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("start_offset", default_value="0.0"),
            DeclareLaunchArgument("play_delay", default_value="20.0",
                                  description="seconds to wait for the nodes before playing"),
            DeclareLaunchArgument("loop", default_value="false", choices=["true", "false"]),
            *[DeclareLaunchArgument(k, default_value=v) for k, v in PERCEPTION_ARGS.items()],
            OpaqueFunction(function=launch_setup),
        ]
    )
