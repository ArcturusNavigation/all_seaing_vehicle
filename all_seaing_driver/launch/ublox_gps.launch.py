import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("all_seaing_driver"),
        "config",
        "zed_f9p.yaml",
    )

    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="both",
        parameters=[params],
    )

    return launch.LaunchDescription(
        [
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
