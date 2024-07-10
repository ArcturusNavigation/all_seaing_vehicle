#!/usr/bin/env python3
import os
import time
import torch
import cv_bridge
import cv2
import PIL
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from ament_index_python.packages import get_package_share_directory


class Yolov5Image(Node):

    def __init__(self):
        super().__init__("yolov5_image")

        self.bridge = cv_bridge.CvBridge()

        # Get pretrained yolov5 models for colored buoys and cardinal markers
        path_hubconfig = os.path.expanduser("~/yolov5")
        # TODO: should be a ros parameter
        model_name = "yolov5s.pt"
        path_model = os.path.join(
            get_package_share_directory("all_seaing_perception"), "models", model_name
        )
        self.model = torch.hub.load(
            path_hubconfig, "custom", path=path_model, source="local"
        )

        # Subscribers and publishers
        qos_profile = QoSProfile(depth=1)
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray, "bounding_boxes", qos_profile
        )
        self.img_pub = self.create_publisher(Image, "image/detections", qos_profile)

        # TODO: should be in a ros timer, and input image should also be a ros parameter
        while not rclpy.shutdown():
            # Get image
            img = PIL.Image.open(
                self.get_package_share_directory("perception_suite")
                + "/images/test_njord_red.jpg"
            )
            img = np.array(img.convert("RGB"))

            self.predict_image(img)

            time.sleep(10)  # Sleep for 10 ms

    def predict_image(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))

        # Get image predictions for colored buoys and cardinal markers
        results = self.model(img)
        preds = results.pandas().xyxy[0]

        self.get_logger().info(str(preds))

        # Set header of bboxes
        bboxes = LabeledBoundingBox2DArray()
        bboxes.header.stamp = self.get_clock().now().to_msg()

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
    image_detector = Yolov5Image()
    rclpy.spin(image_detector)
    image_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
