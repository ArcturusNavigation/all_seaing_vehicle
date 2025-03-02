#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
import numpy as np
import yaml


class ColorSegmentation(Node):

    def __init__(self):
        super().__init__("color_segmentation")

        self.bridge = cv_bridge.CvBridge()

        # Subscribers and publishers
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray, "bounding_boxes", 10
        )
        self.img_pub = self.create_publisher(Image, "image/segmented", 5)
        self.img_sub = self.create_subscription(
            Image,
            "image",
            self.img_callback,
            qos_profile_sensor_data,
        )

        self.declare_parameter('color_ranges_file', '')
        self.declare_parameter('color_label_mappings_file', '')

        color_ranges_file = self.get_parameter('color_ranges_file').value
        color_label_mappings_file = self.get_parameter('color_label_mappings_file').value

        with open(color_ranges_file, 'r') as f:
            self.colors = yaml.safe_load(f)
        with open(color_label_mappings_file, 'r') as f:
            self.label_dict = yaml.safe_load(f)

    def img_callback(self, img):

        bboxes = LabeledBoundingBox2DArray()
        bboxes.header = img.header

        try:
            img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        colors = self.colors
        label_dict = self.label_dict

        rgbs = {
            "red": (255, 0, 0),
            "orange": (255, 165, 0),
            "green": (0, 255, 0),
            "black": (0, 0, 0),
            "white": (255, 255, 255),
        }

        for color in colors:
            if color == "red2":
                continue
            h_min, h_max, s_min, s_max, v_min, v_max = colors[color]
            lower_limit = np.array([h_min, s_min, v_min])
            upper_limit = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv_img, lower_limit, upper_limit)
            if color == "red":
                h_min, h_max, s_min, s_max, v_min, v_max = colors["red2"]
                lower_limit = np.array([h_min, s_min, v_min])
                upper_limit = np.array([h_max, s_max, v_max])
                mask = mask + cv2.inRange(hsv_img, lower_limit, upper_limit)

            # Erode and dilate mask
            kernel2 = np.ones((30, 30), np.uint8)
            mask = cv2.dilate(mask, kernel2, iterations=1)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                bbox = LabeledBoundingBox2D()
                bbox.min_x = x
                bbox.min_y = y
                bbox.max_x = x + w
                bbox.max_y = y + h
                bbox.label = label_dict[color]

                bboxes.boxes.append(bbox)

                # Draw a rectangle on the image that is the bounding box
                cv2.rectangle(
                    img,
                    (bbox.min_x, bbox.min_y),
                    (bbox.max_x, bbox.max_y),
                    rgbs[color],
                    4,
                )

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        self.bbox_pub.publish(bboxes)


def main(args=None):
    rclpy.init(args=args)
    color_segmentation = ColorSegmentation()
    rclpy.spin(color_segmentation)
    color_segmentation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
