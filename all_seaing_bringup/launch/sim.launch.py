from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch_ros
import os


def generate_launch_description():

    vrx_gz_prefix = get_package_share_directory("vrx_gz")
    bringup_prefix = get_package_share_directory("all_seaing_bringup")
    description_prefix = get_package_share_directory("all_seaing_description")
    robot_localization_params = os.path.join(
        bringup_prefix, "config", "robot_localization", "localize_sim.yaml"
    )
    keyboard_params = os.path.join(bringup_prefix, "config", "keyboard_controls.yaml")

    bag_path = LaunchConfiguration("bag_path")
    launch_rviz = LaunchConfiguration("launch_rviz")
    record_bag = LaunchConfiguration("record_bag")

    bag_path_launch_arg = DeclareLaunchArgument("bag_path", default_value=".")
    launch_rviz_launch_arg = DeclareLaunchArgument(
        "launch_rviz", default_value="true", choices=["true", "false"]
    )
    record_bag_launch_arg = DeclareLaunchArgument(
        "record_bag", default_value="true", choices=["true", "false"]
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[robot_localization_params],
    )

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        remappings=[("gps/fix", "/wamv/sensors/gps/gps/fix")],
        parameters=[robot_localization_params],
    )

    protobuf_client_node = launch_ros.actions.Node(
        package="protobuf_client",
        executable="protobuf_client_node",
        output="screen",
    )

    moos_to_controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="moos_to_controller",
    )

    controller_node = launch_ros.actions.Node(
        package="all_seaing_controller",
        executable="xdrive_controller.py",
        parameters=[{"in_sim": True}],
    )

    rviz_testing_helper_node = launch_ros.actions.Node(
        package="all_seaing_utility",
        executable="rviz_testing_helper.py",
        parameters=[{"bag_path": bag_path}],
        condition=IfCondition(record_bag),
    )

    keyboard_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard",
    )

    keyboard_to_joy_node = launch_ros.actions.Node(
        package="keyboard",
        executable="keyboard_to_joy.py",
        parameters=[{"config_file_name": keyboard_params}],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(bringup_prefix, "rviz", "dashboard.rviz"),
        ],
        condition=IfCondition(launch_rviz),
    )

    sim_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
        launch_arguments={
            "world": "sydney_regatta",
            "urdf": f"{description_prefix}/urdf/xdrive_wamv/wamv_target.urdf",
        }.items(),
    )

    return LaunchDescription(
        [
            bag_path_launch_arg,
            launch_rviz_launch_arg,
            record_bag_launch_arg,
            ekf_node,
            navsat_node,
            protobuf_client_node,
            moos_to_controller_node,
            controller_node,
            rviz_testing_helper_node,
            keyboard_node,
            keyboard_to_joy_node,
            rviz_node,
            sim_ld,
        ]
    )