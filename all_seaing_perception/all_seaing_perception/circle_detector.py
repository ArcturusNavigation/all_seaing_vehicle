#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleDetector(Node):
    def __init__(self):
        image_topic = "image_to_detect"
        super().__init__("circle_detector")
        self.create_subscription(Image, image_topic, self.detect_circle, 10)
        self.annotated_publisher = self.create_publisher(Image, "circle_detected_image", 10)
        self.bridge = CvBridge()
        print('started circle detector node')
        self.get_logger().info("This is an info message")
        
    def detect_circle(self, img_msg):
        try:
            # Convert ROS image message to OpenCV image
            cv2_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            # Convert to grayscale
            gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
            # Apply Gaussian Blur
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            # Detect circles using Hough Circle Transform
            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, 
                                       param1=50, param2=30, minRadius=10, maxRadius=100)

            if circles is not None:
                circles = np.uint16(np.around(circles))
                for circle in circles[0, :]:
                    # Draw the circle
                    cv2.circle(cv2_image, (circle[0], circle[1]), circle[2], (0, 255, 0), 2)
                    # Draw the center of the circle
                    cv2.circle(cv2_image, (circle[0], circle[1]), 2, (0, 0, 255), 3)
                    self.get_logger().info("Detected a circle!")

            # Convert the annotated image back to ROS format
            annotated_image = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
            self.annotated_publisher.publish(annotated_image)

        except Exception as e:
            self.get_logger().error(f"Failed to detect circle: {e}")

def main(args=None):
    rclpy.init(args=args)
    circle_detector_node = CircleDetector()
    rclpy.spin(circle_detector_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
