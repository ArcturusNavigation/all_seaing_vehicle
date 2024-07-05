from launch import LaunchDescription
import launch_ros


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.SetParameter(name="fcu_url", value="/dev/ttyACM0"),
            launch_ros.actions.Node(
                package="mavros", executable="mavros_node", output="screen"
            ),
        ]
    )
