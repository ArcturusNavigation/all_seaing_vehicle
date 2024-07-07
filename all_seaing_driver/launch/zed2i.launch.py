from launch import LaunchDescription
import launch_ros


def generate_launch_description():

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        get_package_share_directory("zed_wrapper"),
                        "/launch/zed_camera.launch.py",
                    ]
                ),
                launch_arguments={"camera_model": "zed2i"}.items(),
            )
        ]
    )
