#!/usr/bin/env python3
"""
References:
https://github.com/mgonzs13/yolov8_ros


Instructions (run this in terminal):
ros2 run all_seaing_perception yolov8_node.py --ros-args \
  -p model:=model_name.pt \
  -p device:=cuda:0 \
  -p threshold:=0.5 \
  -p image_topic:=your_custom_image_topic
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
from ultralytics.engine.results import Boxes
from ultralytics.utils.plotting import Annotator

from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

import os
import cv2
import yaml

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class Yolov8Node(Node):


    def __init__(self) -> None:
        super().__init__("yolov8_node")

        perception_prefix = get_package_share_directory("all_seaing_perception")
        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.get_logger().info(perception_prefix)
        self.get_logger().info(bringup_prefix)

        # Declare parameters
        self.declare_parameter("model", "yolov8m_roboboat_current_model")
        self.declare_parameter("device", "cuda:0") # change to cpu if running on laptop w/o cuda
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)

        # Get parameters
        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        yaml_file_path = os.path.join(bringup_prefix, 'config','perception','color_label_mappings.yaml')
        
        with open(yaml_file_path,'r') as f:
            self.label_dict = yaml.safe_load(f)

        # Get the model's path
        engine_path = os.path.join(perception_prefix, 'models', model_name + '.engine')
        pt_path = os.path.join(perception_prefix, 'models', model_name + '.pt')
        self.using_tensorRT = False

        # Initialize YOLO model
        # check if we are using the jetson and if the engine path exists.
        if os.path.exists('/etc/nv_tegra_release') and os.path.isfile(engine_path):
            self.get_logger().info(f"Loading model from tensorRT engine: {engine_path}")
            self.cv_bridge = CvBridge()
            self.model = YOLO(engine_path)
            self.using_tensorRT = True
        elif os.path.isfile(pt_path):
            self.get_logger().info(f"Loading model from pt model: {pt_path}")
            self.cv_bridge = CvBridge()
            self.model = YOLO(pt_path)
        else:
            self.get_logger().error(f"Both model paths do not exist :( TensorRT: {engine_path} and pt: {pt_path}")
            

        # Setup QoS profile
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher and Subscriber
        self._pub = self.create_publisher(LabeledBoundingBox2DArray, "bounding_boxes", 10)
        self._image_pub = self.create_publisher(Image, "annotated_image", 10)
        self._sub = self.create_subscription(Image, image_topic, self.image_cb, image_qos_profile)

        # Service for enabling/disabling
        self.enable = True
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def image_cb(self, msg: Image) -> None:

        if self.enable:
            # Convert image to cv_image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            pred_results = self.model.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = pred_results[0].cpu()

            label_dict = self.label_dict

            # Create labeled_bounding_box msgs
            labeled_bounding_box_msgs = LabeledBoundingBox2DArray()

            for box_data in results.boxes:

                box_msg = LabeledBoundingBox2D()

                if results.boxes:

                    # box_msg.label = int(box_data.cls)
                    box_msg.probability = float(box_data.conf)
                    center_x, center_y, width, height = box_data.xywh[0]
                    box_msg.min_x = int(center_x - width / 2)
                    box_msg.max_x = int(center_x + width / 2)
                    box_msg.min_y = int(center_y - height / 2)
                    box_msg.max_y = int(center_y + height / 2)

                    labeled_bounding_box_msgs.boxes.append(box_msg)

                    annotator = Annotator(cv_image, 2, 2, "Arial.ttf", False)

                    class_name = self.model.names[int(box_data.cls)] + str(int(box_data.cls))
                    class_name_list = class_name.split('_')
                    color_name = class_name_list[0]
                    color = ()
                    text_color = (0,0,0)
                    if color_name == "red":
                        color = (0, 0, 255)
                    elif color_name == "green":
                        color = (0,255,0)
                    elif color_name == "yellow":
                        color = (0,230,230)
                    elif color_name == "orange":
                        color = (0,165,255)
                    elif color_name == "black":
                        color = (0,0,0)
                    elif color_name == "white":
                        color = (255,255,255)

                    if color_name in label_dict:
                        box_msg.label = label_dict[color_name]
                        annotator.box_label((box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y), str(class_name), color, text_color)
                        self.get_logger().debug(f"Detected: {class_name} Msg Label is {box_msg.label}")

            # Publish detections
            self._pub.publish(labeled_bounding_box_msgs)

            # Convert annotated image back to ROS Image message
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self._image_pub.publish(annotated_image_msg)  # Publish annotated image


def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
