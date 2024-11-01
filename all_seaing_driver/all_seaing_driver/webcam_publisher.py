#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")
        self.publisher_ = self.create_publisher(Image, "webcam_image", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.publish_webcam_image)
        self.cap = cv2.VideoCapture(0)

    def publish_webcam_image(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)

    def close(self):
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)  # means run continously
    except KeyboardInterrupt:
        webcam_publisher.close()
        webcam_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
