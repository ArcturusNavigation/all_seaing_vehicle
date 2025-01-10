#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class TriangleDetector(Node):
    def __init__(self):
        image_topic = "image_to_detect"
        super().__init__("triangle_detector")
        self.create_subscription(Image, image_topic, self.detect_triangle, 10)
        self.annotated_publisher = self.create_publisher(Image, "triangle_detected_image", 10)
        self.bridge = CvBridge()
        print('started triangle detector node')
        self.get_logger().info("This is an info message")
        
    def detect_triangle(self, img_msg):
        try:
            # Convert ROS image message to OpenCV image
            cv2_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # Convert to grayscale
            gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
            # Apply Gaussian Blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            # Detect edges using Canny
            edges = cv2.Canny(blurred, 50, 150)
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                # Approximate the contour to a polygon
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                # Check if the polygon has 3 vertices (triangle)
                if len(approx) == 3:
                    # Draw the triangle on the image
                    cv2.drawContours(cv2_image, [approx], -1, (0, 255, 0), 2)
                    self.get_logger().info("this is a triangle!")

            # Convert the annotated image back to ROS format
            annotated_image = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
            self.annotated_publisher.publish(annotated_image)

        except Exception as e:
            self.get_logger().error(f"Failed to detect triangle: {e}")

def main(args=None):
    rclpy.init(args=args)
    triangle_detector_node = TriangleDetector()
    rclpy.spin(triangle_detector_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
