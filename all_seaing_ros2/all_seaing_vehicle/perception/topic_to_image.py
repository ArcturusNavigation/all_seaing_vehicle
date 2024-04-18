#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

bridge = CvBridge()

class TopicToImage(Node):

    def __init__(self):
        super().__init__("topic_to_image")

        self.directory = "./data"  # Path to the directory where images will be saved
        # Ensure the directory exists
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
            print("Created directory: {}".format(self.directory))


        # qos_profile = QoSProfile(depth=1)
        self.img_sub = self.create_subscription(Image, "/wamv/sensors/cameras/front_right_camera_sensor/image_raw", self.img_callback, 10)

    def img_callback(self, msg: Image):
        # Converts a ROS Image topic to a photo (png, jpeg etc) and save it a folder "data" on your machine
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        image_path = os.path.join(self.directory, "current_image.png")
        cv2.imwrite(image_path, cv_image)
        print('Wrote image to: {}'.format(image_path))



def main(args=None):
    rclpy.init(args=args)
    node = TopicToImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
