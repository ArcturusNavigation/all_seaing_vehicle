#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from all_seaing_interfaces.msg import ObstacleMap


class GridMapGenerator(Node):
    """Inputs obstacle convex hulls(labeled_map) and outputs global map (Occupancy_grid)"""

    def __init__(self):
        super().__init__("grid_map_generator")

        # --------------- PARAMETERS ---------------#

        self.global_frame_id = (
            self.declare_parameter("global_frame_id", "map")
            .get_parameter_value()
            .string_value
        )

        self.timer_period = (
            self.declare_parameter("timer_period", 0.4)
            .get_parameter_value()
            .double_value
        )

        default_lidar_range = (
            self.declare_parameter("default_lidar_range", 130.0)
            .get_parameter_value()
            .double_value
        )

        self.obstacle_radius_sigma = (
            self.declare_parameter("obstacle_radius_sigma", 3.0)
            .get_parameter_value()
            .double_value
        )

        self.search_radius_sigma = (
            self.declare_parameter("search_radius_sigma", 5.0)
            .get_parameter_value()
            .double_value
        )

        self.grid_dim = (
            self.declare_parameter("grid_dim", [2000, 2000])  # [width, height]
            .get_parameter_value()
            .integer_array_value
        )

        self.grid_resolution = (
            self.declare_parameter("grid_resolution", 0.3)
            .get_parameter_value()
            .double_value
        )

        self.dynamic_origin = (
            self.declare_parameter("dynamic_origin", False)
            .get_parameter_value()
            .bool_value
        )

        # --------------- SUBSCRIBERS, PUBLISHERS, AND TIMERS ---------------#

        self.obstacle_map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/raw", self.hulls_update_cb, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10,
        )

        self.grid_pub = self.create_publisher(OccupancyGrid, "/dynamic_map", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --------------- MEMBER VARIABLES ---------------#

        self.initialize_grid()
        self.obstacle_map = ObstacleMap()
        self.ship_pos = (0, 0)
        self.lidar_range = default_lidar_range
        self.got_odom = False

    def initialize_grid(self):
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = self.global_frame_id

        self.grid.info = MapMetaData()
        self.grid.info.width = self.grid_dim[0]
        self.grid.info.height = self.grid_dim[1]
        self.grid.info.resolution = self.grid_resolution

        self.grid.data = [-1] * self.grid.info.width * self.grid.info.height
        self.active_cells = [False] * self.grid.info.width * self.grid.info.height

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        origin = self.grid.info.origin.position
        resolution = self.grid.info.resolution
        gx = int((x - origin.x) / resolution)
        gy = int((y - origin.y) / resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates back to world coordinates."""
        origin = self.grid.info.origin.position
        resolution = self.grid.info.resolution
        x = gx * resolution + origin.x
        y = gy * resolution + origin.y
        return x, y

    def set_active(self, make_active):
        """
        Set the active_cells for bounding boxes to be true (or false when resetting)

        Uses the scanline rendering algorithm (also used for rasterization!):
        https://en.wikipedia.org/wiki/Scanline_rendering
        """

        edge_table = {} # ymin: (ymax, x, dx/dy)
        ymin, ymax = self.grid.info.height, 0
        for obstacle in self.obstacle_map.obstacles:
            polygon = obstacle.global_chull.polygon
            for i, low_point in enumerate(polygon.points):
                j = (i + 1) % len(polygon.points)
                high_point = polygon.points[j]
                high_point = self.world_to_grid(high_point.x, high_point.y)
                low_point = self.world_to_grid(low_point.x, low_point.y)

                if high_point[1] == low_point[1]: # get rid of horizontal lines
                    continue

                if low_point[1] > high_point[1]:
                    low_point, high_point = high_point, low_point

                invslope = (high_point[0] - low_point[0]) / (high_point[1] - low_point[1]) 
                edge_table.setdefault(low_point[1], []).append(
                    {
                        "ymax": high_point[1],
                        "x": low_point[0],
                        "inverse_slope": invslope
                    }
                )
                ymin = min(ymin, low_point[1])
                ymax = max(ymax, high_point[1])

        active_edge_table = []
        for y in range(ymin, ymax):
            if y in edge_table:
                active_edge_table.extend(edge_table[y])
            active_edge_table = [edge for edge in active_edge_table if edge["ymax"] > y]
            active_edge_table.sort(key=lambda edge: edge["x"])

            # even odd rule: fill between pairs
            for i in range(0, len(active_edge_table), 2):
                if i + 1 >= len(active_edge_table):
                    break
                x_start = math.ceil(active_edge_table[i]["x"])
                x_end = math.ceil(active_edge_table[i+1]["x"])
                for x in range(x_start, x_end + 1):
                    if 0 <= x < self.grid.info.width and 0 <= y < self.grid.info.height:
                        self.active_cells[x + y * self.grid.info.width] = make_active
                        self.grid.data[x + y * self.grid.info.width] = 100

            for edge in active_edge_table:
                edge["x"] += edge["inverse_slope"]

    def find_active_cells(self):
        """
        Create a Gaussian distribution around obstacles using their radius
        """

        self.set_active(True)
        self.modify_probability()
        self.set_active(False)

    def modify_probability(self):
        """Decay or increase probability of obstacle in active cells based on sensor observations"""
        lidar_range_grid = int(self.lidar_range / self.grid_resolution)
        for x in range(
            max(0, self.ship_pos[0] - lidar_range_grid),
            min(self.grid.info.width, self.ship_pos[0] + lidar_range_grid + 1),
        ):
            for y in range(
                max(0, self.ship_pos[1] - lidar_range_grid),
                min(self.grid.info.height, self.ship_pos[1] + lidar_range_grid + 1),
            ):
                if (x - self.ship_pos[0]) ** 2 + (
                    y - self.ship_pos[1]
                ) ** 2 > lidar_range_grid**2:
                    continue
                curVal = self.grid.data[x + y * self.grid.info.width]
                if curVal == -1:
                    curVal = 0
                if self.active_cells[x + y * self.grid.info.width]:
                    curVal += 1
                    curVal *= 5
                    curVal = min(100, curVal)
                else:
                    curVal /= 1.8 # decrease probability by some small amount
                    curVal = math.floor(curVal)
                self.grid.data[x + y * self.grid.info.width] = curVal

    def timer_callback(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid)

    def hulls_update_cb(self, msg):
        self.obstacle_map = msg
        self.find_active_cells()

    def odom_callback(self, msg):
        self.ship_pos = self.world_to_grid(
            msg.pose.pose.position.x, msg.pose.pose.position.y
        )
        if self.dynamic_origin:
            if not self.got_odom:
                self.grid.info.origin.position.x = msg.pose.pose.position.x - self.grid_dim[0]*self.grid_resolution/2
                self.grid.info.origin.position.y = msg.pose.pose.position.y - self.grid_dim[0]*self.grid_resolution/2
        self.got_odom = True

    def scan_callback(self, msg):
        self.lidar_range = msg.range_max


def main(args=None):
    rclpy.init(args=args)
    node = GridMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
