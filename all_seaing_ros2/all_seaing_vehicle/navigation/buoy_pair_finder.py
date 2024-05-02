#!/usr/bin/env python3
import rclpy
import numpy as np
import math

from functools import cmp_to_key
from rclpy.node import Node
from all_seaing_interfaces.msg import CloudClusterArray
from nav_msgs.msg import Odometry
from protobuf_client_interfaces.msg import Gateway
from tf_transformations import euler_from_quaternion

class BuoyPairFinder(Node):

    def __init__(self):
        super().__init__("buoy_pair_finder")

        self.cluster_sub = self.create_subscription(CloudClusterArray, "/labeled_cloud_clusters", self.buoy_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb, 10)
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)
        self.nav_x = 0
        self.nav_y = 0
        self.nav_heading = 0

    def convert_to_global(self, x, y):
        magnitude = math.hypot(x, y)
        angle = math.atan2(y, x)
        new_x = self.nav_x + math.cos(self.nav_heading + angle) * magnitude
        new_y = self.nav_y + math.sin(self.nav_heading + angle) * magnitude
        return new_x, new_y;

    def odom_cb(self, msg):
        self.nav_x = msg.pose.pose.position.x
        self.nav_y = msg.pose.pose.position.y
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation)
        self.nav_heading = yaw

    def buoy_cb(self, msg):

        # Extract buoy points
        left_buoy_points = []
        right_buoy_points = []
        for cluster in msg.clusters:
            self.get_logger().info(f"nav x: {self.nav_x}, nav y: {self.nav_y}, nav heading {self.nav_heading}")
            self.get_logger().info(f"local x: {cluster.avg_point.x}, local y: {cluster.avg_point.y}")
            x, y = self.convert_to_global(cluster.avg_point.x, cluster.avg_point.y)
            self.get_logger().info(f"global x: {x}, global y: {y}")
            if cluster.label == 2:
                left_buoy_points.append((x, y))
            elif cluster.label == 1:
                right_buoy_points.append((x, y))

        if len(left_buoy_points) == 0 or len(right_buoy_points) == 0:
            return

        # Order by euclidian distance to each point
        left_buoy_points = np.array(
            sorted(
                left_buoy_points,
                key=cmp_to_key(lambda x, y: np.linalg.norm(x) - np.linalg.norm(y)),
            )
        )
        right_buoy_points = np.array(
            sorted(
                right_buoy_points,
                key=cmp_to_key(lambda x, y: np.linalg.norm(x) - np.linalg.norm(y)),
            )
        )

        # Find midpoints
        midpoints = []
        for l_pt, r_pt in zip(left_buoy_points, right_buoy_points):
            midpoint = (l_pt + r_pt) / 2
            midpoints.append(midpoint)

        # Publish midpoints
        wpt_msg = Gateway()
        inner_string = "points="
        for i, midpoint in enumerate(midpoints):
            inner_string += f"{midpoint[0]},{midpoint[1]}"
            if i < len(midpoints) - 1:
                inner_string += ":"
        wpt_msg.gateway_key = "WPT_UPDATE"
        wpt_msg.gateway_string = inner_string
        self.publisher.publish(wpt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyPairFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
