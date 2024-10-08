#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped, Pose
from nav_msgs.msg import OccupancyGrid
from utils import PriorityQueue
# from tf_transformations import euler_from_quaternion, quaternion_from_euler
# from visualization_msgs.msg import Marker, MarkerArray

from math import inf, sqrt #Caution: only in Python 3.5
import time
import numpy as np

# import cProfile

class PathPlan(Node):
    """ Inputs obstacles (OccupancyGrid) and waypoints (PoseArray) and outputs a path to follow (PoseArray)
        using Astar
    """

    def __init__(self):
        super().__init__("astar_path_planner")

        self.map_topic = "map" # occupancy grid
        self.waypoints_topic = "default"
        # 2-D grid map, each cell represents the probability of occupancy
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            10)

        self.get_logger().info("initialized")

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.waypoints_topic,
            self.waypoints_cb,
            10
        )
        self.publisher = self.create_publisher(PoseArray, "path_planning", 10)

        self.map_grid = None
        self.map_info = None #m/cell, resolution of the map
        self.target = None
        self.cutoff = 50

        self.waypoints = None

    def map_cb(self, msg):
        self.get_logger().info("Initialized Map" if self.map_info == None else "Updated Map")
        self.map_info = msg.info
        self.map_grid = msg.data

    def waypoints_cb(self, msg):
        self.get_logger().info("New Waypoints Added, Running Astar")
        self.waypoints = msg
        self.full_path()

    def heuristic(self, node): #currently uses euclidean distance as heuristic
        return sqrt((node[0]-self.target[0])**2 + (node[1]-self.target[1])**2)

    def full_path(self):
        total_path = []
        for i in range(0, self.waypoints.length-1):
            total_path.append(self.plan_path(self.waypoints[i], self.waypoints[i+1]))
        PA = PoseArray()
        PA.poses = total_path
        self.publish_path(PA)


    def plan_path(self, s, t):
        """
        start_point s: Ros2 Pose
        end_point t: Ros2 Pose
        return: Pose array
        astar algorithm
        """
        W = self.map_info.width
        H = self.map_info.height
        dxy = [(1,0), (0,1), (-1,0), (0,-1)]
        # index occupancy grid (self.map_grid) with self.map_grid.data[r*W+c]
        gscore = [inf] * (H*W)
        parent = [(0,0)] * (H*W)
        spos = (s.x, s.y)
        tpos = (t.x, t.y)
        self.target = tpos

        gscore[spos[0]*W + spos[1]] = 0
        parent[spos[0]*W + spos[1]] = spos

        pq = PriorityQueue()
        pq.put((self.heuristic(spos), spos[0], spos[1]))

        while not pq.empty():
            node = pq.get()

            if abs(node[0] - (gscore[node[1]*W+node[2]] + self.heuristic(node[1:3]))) < 0.005 :
                continue
            node = node[1:3]
            if node == tpos:
                break
            for d in dxy:
                nxt = (node[0]+d[0], node[1]+d[1])
                if nxt[0] < 0 or nxt[0] > W or nxt[1] < 0 or nxt[1] > H:
                    continue

                if self.map_grid[nxt[0]*W+nxt[1]] > self.cutoff:
                    continue

                if gscore[node[0] * W + node[1]] + 1 < gscore[nxt[0] * W + nxt[1]]:
                    gscore[nxt[0] * W + nxt[1]] = gscore[node[0] * W + node[1]] + 1
                    parent[nxt[0] * W + nxt[1]] = node
                    pq.put((gscore[nxt[0] * W + nxt[1]] + self.heuristic(nxt), nxt[0], nxt[1]))

        # Backtracing
        path = []
        cur = tpos
        while cur != spos:
            path.append(cur)
            cur = parent[cur[0] * W + cur[1]]
        path.append(spos)
        path = list(reversed(path))
        return path

    def pose_to_string(self, pos):
        return "{" + str(pos.position.x) + ", " + str(pos.position.y) + ", " + str(pos.position.z) + "}"

    def publish_path(self,path):
        self.publisher.publish(path)
        self.get_logger().info(f"Publishing: path from " + self.pose_to_string(self.waypoints[0]) + " to " + self.pose_to_string(self.waypoints[-1]))

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()
