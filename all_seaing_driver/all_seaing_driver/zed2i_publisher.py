#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2


class ZEDPublisher(Node):
    def __init__(self):
        super().__init__("zed_publisher")
        self.publisher_ = self.create_publisher(Image, "zed_image", 10)
        self.bridge = CvBridge()
        self.init_camera()
        self.runtime_parameters = sl.RuntimeParameters()
        self.timer = self.create_timer(1.0 / 30.0, self.publish_zed_image)

    def init_camera(self):
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30

        self.zed = sl.Camera()
        if not self.zed.is_opened():
            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {str(err)}")
                return
            else:
                self.get_logger().info("ZED camera opened successfully")

    def publish_zed_image(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            left_image = sl.Mat()
            self.zed.retrieve_image(left_image, sl.VIEW.LEFT)
            frame = left_image.get_data()
            frame_rgb = cv2.cvtColor(
                frame, cv2.COLOR_RGBA2RGB
            )  # cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            ros_image = self.bridge.cv2_to_imgmsg(
                frame_rgb, "bgr8"
            )  # I know it says bgr, trust it's rgb
            self.publisher_.publish(ros_image)
            self.get_logger().info("Image published successfully")
        else:
            self.get_logger().warning("Failed to grab image from ZED camera")


def main(args=None):
    rclpy.init(args=args)
    zed_publisher = ZEDPublisher()

    try:
        rclpy.spin(zed_publisher)

    finally:
        zed_publisher.zed.close()
        zed_publisher.destroy_node()
        rclpy.shutdown()  # haha i make big change


if __name__ == "__main__":
    main()
