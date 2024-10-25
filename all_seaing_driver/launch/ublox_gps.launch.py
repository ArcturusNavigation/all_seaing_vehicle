import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("all_seaing_driver"),
        "config",
        "zed_f9p.yaml",
    )

    port_launch_arg = DeclareLaunchArgument(
        "port",
        default_value='/dev/ttyACM0',
    )

    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="both",
        parameters=[
            params,
            {"device": LaunchConfiguration("port")},
        ],
    )

    return launch.LaunchDescription(
        [
            port_launch_arg,
            ublox_gps_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=ublox_gps_node,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            ),
        ]
    )
