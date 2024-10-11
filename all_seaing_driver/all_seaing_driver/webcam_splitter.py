#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WebcamSplitter(Node):
    def __init__(self):
        super().__init__("webcam_splitter")
        self.publisher_ = self.create_publisher(Image, "webcam_splitted_image", 10)
        # self.publisher_original_ = self.create_publisher(Image, 'webcam_image', 10)
        self.bridge = CvBridge()
        self.cap=cv2.VideoCapture(0)
        self.timer = self.create_timer(1.0/30.0, self.publish_splitted_image) # 30 FPS

    def publish_splitted_image(self):
        got_output, frame = self.cap.read()
        if got_output:
            width = frame.shape[1]
            # ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # self.publisher_original_.publish(ros_image)
            frame = frame[:, 0 : width // 2]  # split in half
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)
            # if cv2.waitKey(1) & 0xFF == ord("q"):
            #         break
    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    splitted_image_publisher = WebcamSplitter()
    try:
        rclpy.spin(splitted_image_publisher)
    except KeyboardInterrupt:
        splitted_image_publisher.close()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
