#!/usr/bin/env python3
"""
References:
https://github.com/mgonzs13/yolov8_ros

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

from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model", "yolov8m.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.cv_bridge = CvBridge()
        self.yolo = YOLO(model)
        self.yolo.fuse()

        # pubs
        self._pub = self.create_publisher(LabeledBoundingBox2DArray, "bounding_boxes", 10)

        # subs
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb,
            image_qos_profile
        )

        # services
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
