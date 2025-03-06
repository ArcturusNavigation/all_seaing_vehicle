#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv_bridge
import cv2
from sensor_msgs.msg import PointCloud2 # check
from geometry_msgs.msg import Pose
import numpy as np

class DockDetection(Node):
    def __init__(self):
        super().__init__("dock_detection")
        self.X_DIM = 10 # TODO CHANGE
        self.Y_DIM = 10
        self.X_RES = 10
        self.Y_RES = 10

        self.bridge = cv_bridge.CvBridge()

        # subscriptions
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            "pointcloud", # TODO REPLACE
            self.pointcloud_callback,
        )

        # publications
        self.pointcloud_converted_image_pub = self.create_publisher(
            Image,
            "image/pointcloud_converted", # TODO REPLACE
            5,
        )
        self.dock_lines_pub = self.create_publisher(
            Image,
            "image/dock_lines", # TODO REPLACE
            5,
        )
        self.dock_pose = self.create_publisher(
            Pose,
            "dock_pose", # TODO REPLACE
            5,
        )

    def pointcloud_callback(self, pointcloud):
        ''' np --> convert pointcloud to overhead 2d image '''
        points = pointcloud.data
        points = np.array(points)[:,:2] # remove z axis
        points[:, 0] /= self.X_RES
        points[:, 1] /= self.Y_RES
        img = np.zeros((X_DIM, Y_DIM))
        img[points.astype(np.int32)] = 1

        ''' cv2 --> hough lines transform? contour? moments calculation? '''
        lines = cv2.HoughLinesP(img, 1, np.pi / 180, 50, None, 50, 10)
        pass

def main(args=None):
    rclpy.init(args=args)
    dock_detection = DockDetection()
    rclpy.spin(dock_detection)
    dock_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()