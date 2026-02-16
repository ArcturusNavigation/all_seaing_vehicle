#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

DESIRED_CAMERA_NAME = "XWF 1080P PC Camera: XWF 1080P"

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__("webcam_publisher")

        video_index = self.declare_parameter(
            "video_index", 0).get_parameter_value().integer_value

        self.image_pub = self.create_publisher(Image, "webcam_image", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.publish_webcam_image)

        # identify the video_index by the camera name
        video_index = -1

        cameras = os.listdir('/sys/class/video4linux/')
        for camera in sorted(cameras, key=lambda a: int(a[-1])):
            try:
                with open(os.path.join('/sys/class/video4linux/', camera, 'name'), 'r') as f:
                    camera_name = f.readline().strip()
                    self.get_logger().info(f'camera name: {camera_name}')
                    if camera_name == DESIRED_CAMERA_NAME:
                        self.get_logger().info(f'desired camera')
                        try:
                            with open(os.path.join('/sys/class/video4linux/', camera, 'index'), 'r') as fs:
                                video_index = int(fs.readline().strip())
                                self.get_logger().info(f'video index: {video_index}')
                                break
                        except:
                            pass
            except:
                pass

        if video_index != -1:
            self.cap = cv2.VideoCapture(video_index)
        else:
            self.destroy_node()

    def publish_webcam_image(self):
        ret, frame = self.cap.read()
        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(ros_image)

    def close(self):
        self.cap.release()
        


def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        webcam_publisher.close()
        webcam_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
