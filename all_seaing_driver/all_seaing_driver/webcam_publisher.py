#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'webcam_image', 10)
        self.bridge = CvBridge()

    def publish_webcam_image(self):
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if ret:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher_.publish(ros_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    webcam_publisher.publish_webcam_image()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

