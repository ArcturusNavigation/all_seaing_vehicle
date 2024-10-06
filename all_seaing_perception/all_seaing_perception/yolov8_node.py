#!/usr/bin/env python3
"""
References:
https://github.com/mgonzs13/yolov8_ros


Instructions (run this in terminal):
ros2 run all_seaing_perception yolov8_node --ros-args \
  -p model:=yolov8m.pt \
  -p device:=cuda:0 \
  -p threshold:=0.5 \
  -p enable:=true \
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
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # Declare parameters
        self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("image_topic", "image_raw") 
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)

        # Get parameters
        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        self.enable = self.get_parameter("enable").get_parameter_value().bool_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        # Get the model's path
        package_path = get_package_share_directory('all_seaing_perception')
        model_path = os.path.join(package_path, 'models', model_name)

        # Initialize YOLO model
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
        self._sub = self.create_subscription(Image, image_topic, self.image_cb, image_qos_profile)

        # Service for enabling/disabling
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

    def enable_cb(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res


    def image_cb(self, msg: Image) -> None:

        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            # create labeled_bounding_box msgs
            labeled_bouding_box_msgs = LabeledBoundingBox2DArray()

            for box_data in results.boxes:

                box_msg = LabeledBoundingBox2D()

                if results.boxes:
                    # self.yolo.names[int(box_data.cls)] for class labels
                    box_msg.label = int(box_data.cls)
                    box_msg.probability = float(box_data.conf)
                    center_x, center_y, width, height = box_data.xywh
                    box_msg.min_x = int(center_x - width/2)
                    box_msg.max_x = int(center_x + width/2)
                    box_msg.min_y = int(center_y - height/2)
                    box_msg.max_y = int(center_y + height/2)

                labeled_bouding_box_msgs.boxes.append(box_msg)

            # publish detections
            self._pub.publish(labeled_bouding_box_msgs)


def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
