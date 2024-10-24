#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid, Path
from utils import PriorityQueue
from math import inf, sqrt
import numpy as np

class PathPlan(Node):
    """ Inputs obstacles (OccupancyGrid) and waypoints (PoseArray) and outputs a path using A* """

    def __init__(self):
        super().__init__("astar_path_planner")

        self.map_topic = "map"  # OccupancyGrid
        self.waypoints_topic = "waypoints"

        # Subscriptions to map and waypoints
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, 10)

        self.goal_sub = self.create_subscription(
            PoseArray, self.waypoints_topic, self.waypoints_cb, 10)

        # Publishers for path and PoseArray
        self.pose_array_pub = self.create_publisher(PoseArray, "path_planning", 10)
        self.path_pub = self.create_publisher(Path, "a_star_path", 10)

        self.map_grid = None
        self.map_info = None  # Resolution of the map (m/cell)
        self.target = None
        self.cutoff = 50  # Threshold for obstacle cells
        self.waypoints = None
        self.failed_runs = 0
        self.completed_runs = 0

        self.get_logger().debug("Initialized A* Path Planner")

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        gx = int((x - origin.x) / resolution)
        gy = int((y - origin.y) / resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates back to world coordinates."""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        x = gx * resolution + origin.x
        y = gy * resolution + origin.y
        return x, y

    def map_cb(self, msg):
        self.map_info = msg.info
        self.map_grid = msg.data
        self.get_logger().debug("Initialized Map" if self.map_info is None else "Updated Map")

    def waypoints_cb(self, msg):
        self.waypoints = msg
        self.get_logger().debug("New Waypoints Added, Running A*")
        if self.map_info is None:
            self.get_logger().debug("No occupancy grid in memory")
            return
        self.full_path()

    def heuristic(self, node):
        """Euclidean distance as heuristic"""
        return sqrt((node[0] - self.target[0]) ** 2 + (node[1] - self.target[1]) ** 2)

    def full_path(self):
        total_path = []
        for i in range(len(self.waypoints.poses) - 1):
            total_path.extend(self.plan_path(self.waypoints.poses[i], self.waypoints.poses[i + 1]))

        # Convert total path to PoseArray for publishing
        pose_array = PoseArray()
        for position in total_path:
            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose_array.poses.append(pose)

        # self.publish_path(pose_array)

    def plan_path(self, s, t):
        """A* Algorithm to compute the path"""
        W = self.map_info.width
        H = self.map_info.height

        dxy = [(1, 0), (0, 1), (-1, 0), (0, -1),(1,1),(-1,-1),(-1,1),(1,-1),
                (1,-2), (1,2), (-1, 2), (-1,-2), (2,1), (-2,1), (2,-1), (-2,-1)] # Neighbor offsets

        gscore = [inf] * (H * W)
        parent = [(0, 0)] * (H * W)

        # Convert waypoints to grid coordinates
        spos = self.world_to_grid(s.position.x, s.position.y)
        tpos = self.world_to_grid(t.position.x, t.position.y)
        self.target = tpos

        # Check if the starting position and/or the ending position is occupied
        if (self.map_grid[spos[0] + spos[1] * W] > self.cutoff or self.map_grid[spos[0] + spos[1] * W] == -1):
            self.map_grid[spos[0] + spos[1] * W] = 0
        if (self.map_grid[tpos[0] + tpos[1] * W] > self.cutoff or self.map_grid[tpos[0] + tpos[1] * W] == -1):
            self.map_grid[tpos[0] + tpos[1] * W] = 0

        gscore[spos[0] + spos[1] * W] = 0
        parent[spos[0] + spos[1] * W] = spos

        pq = PriorityQueue()
        pq.put((self.heuristic(spos), spos[0], spos[1]))
        while not pq.empty():
            node = pq.get()
            if abs(node[0] - (gscore[node[1] + node[2] * W] + self.heuristic(node[1:3]))) > 0.005:
                continue

            node = node[1:3]
            if node == tpos:
                break

            for d in dxy:
                skip = False
                nxt = (node[0] + d[0], node[1] + d[1])
                if nxt[0] < 0 or nxt[0] >= H or nxt[1] < 0 or nxt[1] >= W:
                    continue
                if d[0]**2+d[1]**2 == 2 or d[0]**2+d[1]**2 == 5:
                    for tx in range(min(node[0], nxt[0]), max(node[0],nxt[0])+1):
                        for ty in range(min(node[1],nxt[1]), max(node[1], nxt[1])+1):
                            if self.map_grid[tx + ty * W] > self.cutoff or self.map_grid[tx + ty * W] == -1:
                                skip = True
                if skip:
                    continue

                if self.map_grid[nxt[0] + nxt[1] * W] > self.cutoff or self.map_grid[nxt[0] + nxt[1] * W] == -1:
                    continue

                if gscore[node[0] + node[1] * W] + sqrt(d[0]**2+d[1]**2) < gscore[nxt[0] + nxt[1] * W]:
                    gscore[nxt[0]+ nxt[1] * W] = gscore[node[0] + node[1] * W] + sqrt(d[0]**2+d[1]**2)
                    parent[nxt[0] + nxt[1] * W] = node
                    pq.put((gscore[nxt[0]+ nxt[1] * W] + self.heuristic(nxt), nxt[0], nxt[1]))

        if gscore[tpos[0] + tpos[1]*W] == inf:
            self.get_logger().debug("Error: Path not found")
            path = []
            self.failed_runs += 1
            return path

        # Backtrace the path
        path = []
        cur = tpos
        while cur != spos:
            path.append(cur)
            cur = parent[cur[0] + cur[1] * W]
        path.append(spos)
        path.reverse()
        self.get_logger().debug("Finished Backtracking Path")

        self.completed_runs += 1
        self.get_logger().debug(f"Failed runs/Completed runs: {self.failed_runs}/{self.completed_runs}")

        # Publish path as nav_msgs/Path
        self.publish_nav_path(path)
        return path

    def publish_nav_path(self, path):
        """Publish path as nav_msgs/Path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in path:
            wx, wy = self.grid_to_world(point[0], point[1])  # Convert grid to world coordinates
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().debug("Published A* Path")

    def pose_to_string(self, pos):
        return f"{{{pos.position.x}, {pos.position.y}, {pos.position.z}}}"

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
