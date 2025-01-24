#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Odometry, PointCloud2
import numpy as np


class DockDetecor(Node):

    def __init__(self):
        super().__init__("color_segmentation")


        # Subscribers and publishers
        
        # self.img_pub = self.create_publisher(Image, "image/segmented", 5)
       
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "point_cloud", self.point_cloud_cb, qos_profile_sensor_data
        )
        
        # self.declare_parameter("lidar_point_cloud", "")
        self.declare_parameter('dock_point_cloud_file', '')
        dock_point_cloud_file = self.get_parameter('dock_point_cloud_file').value
        self.known_dock_pc = None
        self.lidar_point_cloud = None
        self.robot_pos = (0, 0)
        self.timer = self.create_timer(1, self.detect_dock)

        with open(dock_point_cloud_file, 'r') as f:
            # self.known_dock_pc =
           pass 
           # store the known point cloud of the dock from f
    def get_point_cloud_transform(self, point_cloud_1, point_cloud_2):
        pass
        # use ICP here
        # return transform and confidence
    def detect_dock(self):
        pass
        # self.lidar_point_cloud_flat = # flatten
        # self.get_point_cloud_transform(__, __)
        # publish something


    def odometry_cb(self, msg):
        """
        Update the stored robot's position based on the odometry messages
        """
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = msg.data # TODO convert to actual numpy array




def main(args=None):
    rclpy.init(args=args)
    dock_detector = DockDetecor()
    rclpy.spin(dock_detector)
    dock_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
