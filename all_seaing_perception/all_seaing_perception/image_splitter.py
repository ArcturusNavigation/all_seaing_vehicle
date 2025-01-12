#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

class ImageSplitter(Node):
    def __init__(self):
        image_topic = "image_to_split"

        super().__init__("image_splitter")
        self.create_subscription(Image, image_topic, self.split_image, 10)
        self.left_publisher = self.create_publisher(Image, "split_image_left", 10)
        self.right_publisher = self.create_publisher(Image, "split_image_right", 10)

        self.bridge = CvBridge()
        print('started image splitter node')

    def split_image(self, img_msg):
        try:
            cv2_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8") # get image in opencv format
            width = cv2_image.shape[1]

            left_cv2 = cv2_image[:, :width//2]  # left half
            right_cv2 = cv2_image[:, width//2:]  # right half

            # convert back to ros format
            left_ros = self.bridge.cv2_to_imgmsg(left_cv2, "bgr8")
            right_ros = self.bridge.cv2_to_imgmsg(right_cv2, "bgr8")
            
            self.left_publisher.publish(left_ros)
            self.right_publisher.publish(right_ros)
        except:
            print("image_splitter failed")

def main(args=None):
    rclpy.init(args=args)

    webcam_splitter_node = ImageSplitter()
    rclpy.spin(webcam_splitter_node)
    rclpy.shutdown()

# ros2 run all_seaing_perception image_splitter.py --ros-args --remap /image_to_split:=/zed/zed_node/rgb/image_rect_color
if __name__ == "__main__":
    main()
