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
        self.get_logger().info("Circle detector node initialized")

    def detect_circle(self, img_msg):
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
                # Calculate area and perimeter
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                
                # For a circle, area/perimeter ratio should be close to r/2
                # And circularity = 4*pi*area/perimeter^2 should be close to 1
                if area > 0 and perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if 0.8 < circularity < 1.2:  # Allow some tolerance
                        cv2.drawContours(cv2_image, [contour], -1, (0, 255, 0), 2)
                        self.get_logger().info("Circle detected!")

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
