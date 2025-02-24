#!/usr/bin/env python3
"""
References:
https://github.com/mgonzs13/yolov8_ros


Instructions (run this in terminal):
ros2 run all_seaing_perception yolov8_node.py \
    --ros-args -p model:=model_name (without .pt) \
    --ros-args -p label_config:=config_in_perception_bringup.yaml \
    --ros-args -p image_topic:=your_custom_image_topic (without /)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.utils.plotting import Annotator

from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

import os
import torch
import yaml

from sensor_msgs.msg import Image


class Yolov8Node(Node):


    def __init__(self) -> None:
        super().__init__("yolov8_node")

        perception_prefix = get_package_share_directory("all_seaing_perception")
        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.get_logger().info(perception_prefix)
        self.get_logger().info(bringup_prefix)

        # Declare parameters
        self.declare_parameter("model", "yolov8m_roboboat_current_model")
        self.declare_parameter("device", "default")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("label_config", "buoy_label_mappings.yaml")
        self.declare_parameter("image_topic", "")

        # Get parameters
        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        self.image_topic_name = self.get_parameter("image_topic").get_parameter_value().string_value

        if self.device == "default":
            self.device = "cuda"  if torch.cuda.is_available() else "cpu"

        label_config = self.get_parameter("label_config").get_parameter_value().string_value
        yaml_file_path = os.path.join(bringup_prefix, 'config','perception', label_config)
        
        with open(yaml_file_path,'r') as f:
            self.label_dict = yaml.safe_load(f)

        # Get the model's path
        engine_path = os.path.join(perception_prefix, 'models', model_name + '.engine')
        pt_path = os.path.join(perception_prefix, 'models', model_name + '.pt')

        # Initialize YOLO model
        # check if we are using the jetson and if the engine path exists.
        self.cv_bridge = CvBridge()
        if os.path.exists('/etc/nv_tegra_release') and os.path.isfile(engine_path):
            self.get_logger().info(f"Loading model from tensorRT engine: {engine_path}")
            self.model = YOLO(engine_path)
        elif os.path.isfile(pt_path):
            self.get_logger().info(f"Loading model from pt model: {pt_path}")
            self.model = YOLO(pt_path)
        else:
            self.get_logger().error(f"Both model paths do not exist :( TensorRT: {engine_path} and pt: {pt_path}")
        # Setup QoS profile
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher and Subscriber
        self._pub = self.create_publisher(LabeledBoundingBox2DArray, "bounding_boxes", 10)
        self._image_pub = self.create_publisher(Image, "annotated_image", 10)
        self._sub = self.create_subscription(Image, self.image_topic_name, self.image_cb, image_qos_profile)

    def image_cb(self, msg: Image) -> None:

        # Convert image to cv_image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        annotator = Annotator(cv_image, 2, 2, "Arial.ttf", False)

        pred_results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.threshold,
            device=self.device
        )
        results: Results = pred_results[0].cpu()

        label_dict = self.label_dict

        labeled_bounding_box_msgs = LabeledBoundingBox2DArray()
        labeled_bounding_box_msgs.header = msg.header

        for box_data in results.boxes:

            box_msg = LabeledBoundingBox2D()

            if results.boxes:

                box_msg.probability = float(box_data.conf)
                center_x, center_y, width, height = box_data.xywh[0]
                box_msg.min_x = int(center_x - width / 2)
                box_msg.max_x = int(center_x + width / 2)
                box_msg.min_y = int(center_y - height / 2)
                box_msg.max_y = int(center_y + height / 2)
                label_name = self.model.names[int(box_data.cls)]
                box_msg.label = int(box_data.cls)
                labeled_bounding_box_msgs.boxes.append(box_msg)
                


                class_name = f"{label_name}_{str(int(box_data.cls))}"
                class_name_list = class_name.split('_')
                color_name = class_name_list[0] # assumes labels follow 'color_object' format
                color = ()
                text_color = (0,0,0)
                if color_name == "red":
                    color = (0, 0, 255)
                elif color_name == "green":
                    color = (0,255,0)
                elif color_name == "blue":
                    color = (255, 0, 0)
                elif color_name == "yellow":
                    color = (0,230,230)
                elif color_name == "black":
                    color = (0,0,0)
                elif color_name == "white":
                    color = (255,255,255)
                else: 
                    color = (128, 128, 128)

                if label_name in label_dict:
                    annotator.box_label((box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y), class_name, color, text_color)
                    self.get_logger().debug(f"Detected: {class_name} Msg Label is {box_msg.label}")
                else: 
                    annotator.box_label((box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y), class_name, color, text_color)
                    self.get_logger().debug(f'Detected: {class_name} Item not in label_dict')

        # Publish detections
        self._pub.publish(labeled_bounding_box_msgs)

        # Convert annotated image back to ROS Image message
        annotated_frame = annotator.result()  
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self._image_pub.publish(annotated_image_msg)  # Publish annotated image


def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
