from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
Reference: https://github.com/mgonzs13/yolov8_ros

Instructions: 
$ cd ~/arcturus/dev_ws/src/
$ git clone https://github.com/mgonzs13/yolov8_ros.git
$ pip install -r yolov8_ros/requirements.txt
$ cd ~/arcturus/dev_ws/
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --symlink-install
$ source ~/arcturus/dev_ws/install/setup.bash
$ source ~/arcturus/vrx_ws/install/setup.bash

follow those instructions before running script

Parameters to Yolov8 Launch
    model: YOLOv8 model (default: yolov8m.pt)
    tracker: tracker file (default: bytetrack.yaml)
    device: GPU/CUDA (default: cuda:0)
    enable: wether to start YOLOv8 enabled (default: True)
    threshold: detection threshold (default: 0.5)
    input_image_topic: camera topic of RGB images (default: /camera/rgb/image_raw)
    image_reliability: reliability for the image topic: 0=system default, 1=Reliable, 2=Best Effort (default: 2)

Ensure that the model parameter has the correct path to a yolov8 model
and that input_image_topic subscribes to the correct topic (such as zed)
"""


def generate_launch_description():

    yolov8_prefix = get_package_share_directory("yolov8_bringup")
    model_share_path = get_package_share_directory("all_seaing_perception")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [yolov8_prefix, "/launch/yolov8.launch.py"]
                ),
                launch_arguments={
                    "model": model_share_path + "/models/yolov8_roboboat_model.pt",
                    "input_image_topic": "/webcam_image",
                }.items(),
            )
        ]
    )
