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

            # Find contours - use RETR_LIST to get both external and internal contours
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            found_cross = False  # Flag to check if cross was detected

            # Draw all contours in blue for debugging
            cv2.drawContours(cv2_image, contours, -1, (255, 0, 0), 2)

            # Display number of vertices for each contour
            for i, contour in enumerate(contours):
                # Get the contour approximation with a much smaller epsilon
                perimeter = cv2.arcLength(contour, True)
                epsilon = 0.02 * perimeter  # Using fixed small epsilon for better corner detection
                approx = cv2.approxPolyDP(contour, epsilon, True)
                num_vertices = len(approx)

                # Calculate centroid of contour for text placement
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw vertex count near the contour
                    cv2.putText(cv2_image, f"n={num_vertices}", (cx, cy),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(cv2_image, f"n={num_vertices}", (cx, cy),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                    # Draw the approximated points in yellow
                    for point in approx:
                        pt = point[0]
                        cv2.circle(cv2_image, (int(pt[0]), int(pt[1])), 3, (0, 255, 255), -1)

            # For actual cross detection, we need to look at the contour shape more carefully
            for contour in contours:
                # Filter small contours
                area = cv2.contourArea(contour)
                if area < 100:  # Adjust this threshold based on your image size
                    continue

                # Get the contour approximation with a much smaller epsilon
                perimeter = cv2.arcLength(contour, True)
                epsilon = 0.02 * perimeter  # Using fixed small epsilon for better corner detection
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if len(approx) == 12:  # Exactly 12 vertices for a cross
                    # Get the minimum area rectangle to check aspect ratio
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    # Calculate aspect ratio
                    width = np.linalg.norm(box[0] - box[1])
                    height = np.linalg.norm(box[1] - box[2])
                    aspect_ratio = min(width, height) / max(width, height)

                    # Cross should have a relatively square bounding box
                    if aspect_ratio > 0.6:  # Cross should be roughly square
                        # Check angles between adjacent edges
                        pts = np.array(approx).reshape(-1, 2)
                        valid_angles = True

                        for i in range(len(pts)):
                            prev_point = pts[i - 1]
                            curr_point = pts[i]
                            next_point = pts[(i + 1) % len(pts)]

                            # Calculate vectors
                            vec1 = prev_point - curr_point
                            vec2 = next_point - curr_point

                            # Normalize vectors
                            vec1 = vec1 / np.linalg.norm(vec1)
                            vec2 = vec2 / np.linalg.norm(vec2)

                            # Calculate angle in degrees
                            dot_product = np.clip(np.dot(vec1, vec2), -1.0, 1.0)
                            angle_rad = np.arccos(dot_product)
                            angle_deg = np.degrees(angle_rad)

                            # Check if angle is between 30 and 120 degrees
                            if not (30 <= angle_deg <= 120):
                                valid_angles = False
                                break

                        if valid_angles:
                            cv2.drawContours(cv2_image, [contour], -1, (0, 255, 0), 2)
                            found_cross = True
                            self.get_logger().info("Cross detected")

            # Log if no valid cross was found
            if not found_cross:
                self.get_logger().info("No valid cross detected.")

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
