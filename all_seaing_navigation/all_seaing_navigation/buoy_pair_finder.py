#!/usr/bin/env python3
import rclpy
import numpy as np
import math

from functools import cmp_to_key
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap
from nav_msgs.msg import Odometry
from protobuf_client_interfaces.msg import Gateway


class BuoyPairFinder(Node):

    def __init__(self):
        super().__init__("buoy_pair_finder")

        self.map_sub = self.create_subscription(
            ObstacleMap, "/labeled_map", self.buoy_cb, 10
        )
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)
        
    def buoy_cb(self, msg):

        # Extract buoy points
        left_buoy_points = []
        right_buoy_points = []
        for obstacle in msg.obstacles:
            x, y = obstacle.global_point.x, obstacle.global_point.y
            if obstacle.label == 2:
                left_buoy_points.append((x, y))
            elif obstacle.label == 1:
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
