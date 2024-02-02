#!/usr/bin/env python3
import rclpy
import numpy as np

from functools import cmp_to_key
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

# TODO: Will input most likely a custom message with ROS msg points (centroids of pointcloud clusters)

class BuoyPairFinder(Node):

    def __init__(self):
        super().__init__("buoy_pair_finder")

        # TODO: Add subscriber
        self.wpt_pub = self.create_publisher(PoseArray, "/waypoints", 10)

        # TODO: Remove timer and instad use subscriber (add msg field)
        self.create_timer(1, self.buoy_cb)

    def buoy_cb(self):
        
        # TODO: Actually use msg instead of hard-coding. (0, 0) is where the boat is.
        left_buoy_points = [(5, 30), (5, 20), (5, 40)]
        right_buoy_points = [(15, 40), (15, 20), (15, 50), (15, 30)]
        
        # Order by euclidian distance to each point
        left_buoy_points = np.array(sorted(left_buoy_points, key=cmp_to_key(lambda x, y: np.linalg.norm(x) - np.linalg.norm(y))))
        right_buoy_points = np.array(sorted(right_buoy_points, key=cmp_to_key(lambda x, y: np.linalg.norm(x) - np.linalg.norm(y))))

        # Find midpoints
        midpoints = PoseArray()
        for l_pt, r_pt in zip(left_buoy_points, right_buoy_points):
            midpoint = (l_pt + r_pt) / 2
            pose = Pose()
            pose.position.x = midpoint[0]
            pose.position.y = midpoint[1]
            midpoints.poses.append(pose)

        # Publish midpoints
        self.wpt_pub.publish(midpoints)

def main(args=None):
    rclpy.init(args=args)
    node = BuoyPairFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
