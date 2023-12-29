#!/usr/bin/env python3
"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/

List of classes:
https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml
"""

import getpass
import torch
import cv_bridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from ament_index_python.packages import get_package_share_directory

class Yolov5Detector(Node):

    def __init__(self):
        super().__init__('yolov5_detector')
        
        self.bridge = cv_bridge.CvBridge()

        # Get pretrained yolov5 models for colored buoys and cardinal markers
        path_hubconfig = f"/home/{getpass.getuser()}/yolov5"
        path_model = get_package_share_directory("perception_suite") + "/models/buoy_detection_best_weights_v7.pt"
        self.model = torch.hub.load(path_hubconfig, 'custom', path=path_model, source='local')

        # Subscribers and publishers
        qos_profile = QoSProfile(depth=1)
        self.bbox_pub = self.create_publisher(LabeledBoundingBox2DArray, '/perception_suite/bounding_boxes', qos_profile)
        self.img_pub = self.create_publisher(Image, '/perception_suite/segmented_image', qos_profile)
        self.img_sub = self.create_subscription(
            Image, 
            '/perception_suite/image', # Remap this to correct topic
            self.img_callback, 
            qos_profile
        )

    def img_callback(self, img):
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
            
            bbox.min_x = int(row['xmin'])
            bbox.min_y = int(row['ymin'])
            
            bbox.max_x = int(row['xmax'])
            bbox.max_y = int(row['ymax'])
            
            bbox.label = row['class']
            bbox.probability = row['confidence']
            bboxes.boxes.append(bbox)
            cv2.rectangle(img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.bbox_pub.publish(bboxes)

def main(args=None):
    rclpy.init(args=args)
    detector = Yolov5Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
