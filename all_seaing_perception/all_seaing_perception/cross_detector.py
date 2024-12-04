#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CrossDetector(Node):
    def __init__(self):
        super().__init__("cross_detector")
        
        # Declare parameters with default values
        self.declare_parameter("edge_threshold1", 50)
        self.declare_parameter("edge_threshold2", 150)
        self.declare_parameter("epsilon_factor", 0.04)
        
        image_topic = "image_to_detect"
        self.create_subscription(Image, image_topic, self.detect_cross, 10)
        self.annotated_publisher = self.create_publisher(Image, "cross_detected_image", 10)
        self.bridge = CvBridge()

        self.get_logger().info('Started cross detector node')

    def detect_cross(self, img_msg):
        try:
            # Convert ROS image message to OpenCV image
            cv2_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian Blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Get Canny edge thresholds from parameters
            edge_threshold1 = self.get_parameter("edge_threshold1").get_parameter_value().integer_value
            edge_threshold2 = self.get_parameter("edge_threshold2").get_parameter_value().integer_value
            edges = cv2.Canny(blurred, edge_threshold1, edge_threshold2)
            
            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            found_cross = False  # Flag to check if any cross was detected
            
            for contour in contours:
                # Approximate the contour to a polygon
                epsilon_factor = self.get_parameter("epsilon_factor").get_parameter_value().double_value
                epsilon = epsilon_factor * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # Check if the polygon has 4 vertices (potential cross)
                if len(approx) == 4:
                    # Further check if the shape is a cross-like figure by examining angles
                    # Sort the points to detect perpendicular arms (cross-like)
                    pts = np.array(approx).reshape(-1, 2)
                    rect = cv2.minAreaRect(pts)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    # Calculate angles between the edges of the bounding box
                    angle1 = np.arctan2(box[1][1] - box[0][1], box[1][0] - box[0][0])  # Angle between points 0-1
                    angle2 = np.arctan2(box[2][1] - box[1][1], box[2][0] - box[1][0])  # Angle between points 1-2
                    
                    # Check if the angles are close to 90 degrees
                    angle_diff = abs(angle1 - angle2)
                    if np.pi/4 < angle_diff < 3*np.pi/4:  # Check for near right angles
                        # Draw the cross on the image
                        cv2.drawContours(cv2_image, [box], -1, (0, 255, 0), 2)
                        found_cross = True
                        self.get_logger().info("Cross detected!")

            # Log if no cross was found
            if not found_cross:
                self.get_logger().info("No cross detected.")
            
            # Convert the annotated image back to ROS format
            annotated_image = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
            self.annotated_publisher.publish(annotated_image)

        except Exception as e:
            self.get_logger().error(f"Failed to detect cross: {e}")

def main(args=None):
    rclpy.init(args=args)
    cross_detector_node = CrossDetector()
    rclpy.spin(cross_detector_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
