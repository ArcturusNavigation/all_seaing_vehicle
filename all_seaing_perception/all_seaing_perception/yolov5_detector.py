#!/usr/bin/env python3
"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/

List of classes:
https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml
"""

import os
import torch
import cv_bridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from ament_index_python.packages import get_package_share_directory


class Yolov5Node(Node):

    def __init__(self):
        super().__init__("yolov5_node")

        self.bridge = cv_bridge.CvBridge()

        path_hubconfig = os.path.expanduser("~/yolov5")
        model_name = "yolov5s.pt"
        path_model = os.path.join(
            get_package_share_directory("all_seaing_perception"), "models", model_name
        )
        self.model = torch.hub.load(
            path_hubconfig, "custom", path=path_model, source="local"
        )

        # Subscribers and publishers
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray, "bounding_boxes", 10
        )
        self.img_pub = self.create_publisher(Image, "image/detections", 5)
        self.img_sub = self.create_subscription(
            Image,
            "image",
            self.img_callback,
            qos_profile_sensor_data,
        )

    def img_callback(self, img):

        # Set header of bboxes
        bboxes = LabeledBoundingBox2DArray()
        bboxes.header = img.header

        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))

        # Get image predictions for colored buoys and cardinal markers
        results = self.model(img)
        preds = results.pandas().xyxy[0]

        self.get_logger().info(str(preds))

        # Set bounding boxes around colored buoys
        for _, row in preds.iterrows():
            bbox = LabeledBoundingBox2D()

            bbox.min_x = int(row["xmin"])
            bbox.min_y = int(row["ymin"])

            bbox.max_x = int(row["xmax"])
            bbox.max_y = int(row["ymax"])

            bbox.label = row["class"]
            bbox.probability = row["confidence"]
            bboxes.boxes.append(bbox)
            cv2.rectangle(
                img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4
            )

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.bbox_pub.publish(bboxes)


def main(args=None):
    rclpy.init(args=args)
    node = Yolov5Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
