#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CirclePublisher(Node):
    def __init__(self, image_path):
        super().__init__("test_circle_publisher")
        self.publisher = self.create_publisher(Image, "image_to_detect", 10)
        self.bridge = CvBridge()
        # Load the single image
        if not os.path.exists(image_path):
            self.get_logger().error(f"Image file not found: {image_path}")
            raise FileNotFoundError(f"Image file not found: {image_path}")
        self.image = cv2.imread(image_path)
        # Publish the image at 1Hz
        self.timer = self.create_timer(1.0, self.publish_image)
        print(f"Started publishing image: {image_path}")

    def publish_image(self):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
            self.publisher.publish(ros_image)
            print("Published the image.")
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    # Specify the image filename
    image_path = "/home/arcturus/arcturus/dev_ws/src/all_seaing_vehicle/all_seaing_perception/all_seaing_perception/circle.jpg"
    circle_publisher = CirclePublisher(image_path)
    rclpy.spin(circle_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()