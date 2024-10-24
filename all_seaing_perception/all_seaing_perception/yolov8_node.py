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

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes

from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

import os
import cv2

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # Declare parameters
        self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("image_topic", "image_raw") 
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)

        # Get parameters
        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        # Get the model's path
        self.model_dir = os.path.expanduser("~/arcturus/dev_ws/src/all_seaing_vehicle/all_seaing_perception/models")
        model_path = os.path.join(self.model_dir, model_name)

        # Initialize YOLO model
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Model file does not exist at path: {model_path}")
        else:
            self.get_logger().info(f"Loading model from: {model_path}")
            self.cv_bridge = CvBridge()
            self.yolo = YOLO(model_path)
            self.yolo.fuse()

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

        self.get_logger().info(f"Yolov8Node initialized")

    def enable_cb(self, req: SetBool.Request, res: SetBool.Response) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def image_cb(self, msg: Image) -> None:

        if self.enable:
            # Convert image to cv_image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")

            # Predict based on image
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            # Create labeled_bounding_box msgs
            labeled_bounding_box_msgs = LabeledBoundingBox2DArray()

            for box_data in results.boxes:

                box_msg = LabeledBoundingBox2D()

                if results.boxes:

                    box_msg.label = int(box_data.cls)
                    box_msg.probability = float(box_data.conf)
                    center_x, center_y, width, height = box_data.xywh[0]
                    box_msg.min_x = int(center_x - width / 2)
                    box_msg.max_x = int(center_x + width / 2)
                    box_msg.min_y = int(center_y - height / 2)
                    box_msg.max_y = int(center_y + height / 2)

                    labeled_bounding_box_msgs.boxes.append(box_msg)

                    # Draw the bounding box on the image
                    cv2.rectangle(cv_image, 
                                  (box_msg.min_x, box_msg.min_y), 
                                  (box_msg.max_x, box_msg.max_y), 
                                  (0, 255, 0),  # Green color
                                  2)  # Thickness

                    class_name = self.yolo.names[box_msg.label]
                    self.get_logger().debug(f"Detected: {class_name}")

            # Publish detections
            self._pub.publish(labeled_bounding_box_msgs)

            # Convert annotated image back to ROS Image message
            annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="rgb8")
            self._image_pub.publish(annotated_image_msg)  # Publish annotated image


def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
