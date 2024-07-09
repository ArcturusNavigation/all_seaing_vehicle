#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
import numpy as np


class ColorSegmentation(Node):

    def __init__(self):
        super().__init__("color_segmentation")

        self.bridge = cv_bridge.CvBridge()

        # Subscribers and publishers
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray, "bounding_boxes", 1
        )
        self.img_pub = self.create_publisher(Image, "segmented_image", 1)
        self.img_sub = self.create_subscription(
            Image,
            "image",
            self.img_callback,
            1,
        )

    def img_callback(self, img):

        bboxes = LabeledBoundingBox2DArray()
        bboxes.header = img.header

        try:
            img = self.bridge.imgmsg_to_cv2(img, "rgb8")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        colors = {
            "orange": [10, 31, 125, 255, 60, 255],
            "red": [0, 10, 175, 255, 70, 255],
            "red2": [170, 179, 175, 255, 70, 255],
            "green": [59, 82, 142, 255, 30, 255],
            "black": [0, 0, 0, 0, 0, 50],
            "white": [0, 0, 0, 10, 200, 255],
            # "default": [0, 179, 0, 255, 0, 255]
        }

        rgbs = {
            "red": (255, 0, 0),
            "orange": (255, 165, 0),
            "green": (0, 255, 0),
            "black": (0, 0, 0),
            "white": (255, 255, 255),
        }

        # TEMPORARY NUMBERS!!!!
        label_dict = {"orange": 0, "red": 1, "green": 2, "black": 3, "white": 4}

        # change later !!!
        # threshold_pixels = 1000

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

            # findContours on mask
            # imgray = cv2.cvtColor(mask, cv.COLOR_BGR2GRAY)
            # ret, thresh = cv2.threshold(mask, 127, 255, 0)

            # erode and dilate mask
            # Define structuring element
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            kernel2 = np.ones((7, 7), np.uint8)
            mask = cv2.dilate(mask, kernel2, iterations=1)

            contours, hierarchy = cv2.findContours(
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

                # draw a rectangle on the image that is the bounding box
                cv2.rectangle(
                    img,
                    (bbox.min_x, bbox.min_y),
                    (bbox.max_x, bbox.max_y),
                    rgbs[color],
                    4,
                )

            self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.bbox_pub.publish(bboxes)

        # num_valid = cv2.countNonZero(mask)

        # if num_valid > threshold_pixels:
        #     result = cv2.bitwise_and(img, img, mask=mask)
        #     result_dict[color] = result


def main(args=None):
    rclpy.init(args=args)
    color_segmentation = ColorSegmentation()
    rclpy.spin(color_segmentation)
    color_segmentation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
