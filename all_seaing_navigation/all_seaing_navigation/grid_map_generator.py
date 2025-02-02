#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

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
            self.declare_parameter("timer_period", 1.0)
            .get_parameter_value()
            .double_value
        )

        default_lidar_range = (
            self.declare_parameter("default_lidar_range", 130.0)
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

    def find_active_cells(self):
        """
        Mark cells inside bbox of each obstacle as active,
        Then modifies probability of each cell based on active/not
        """
        for obstacle in self.obstacle_map.obstacles:
            minx, miny = self.world_to_grid(
                obstacle.global_bbox_min.x, obstacle.global_bbox_min.y
            )
            maxx, maxy = self.world_to_grid(
                obstacle.global_bbox_max.x, obstacle.global_bbox_max.y
            )
            for x in range(max(0, minx - 1), min(self.grid.info.width, maxx + 1)):
                for y in range(max(0, miny - 1), min(self.grid.info.height, maxy + 1)):
                    self.active_cells[x + y * self.grid.info.width] = True
                    self.grid.data[x + y * self.grid.info.width] = 100

        self.modify_probability()

        for obstacle in self.obstacle_map.obstacles:
            minx, miny = self.world_to_grid(
                obstacle.global_bbox_min.x, obstacle.global_bbox_min.y
            )
            maxx, maxy = self.world_to_grid(
                obstacle.global_bbox_max.x, obstacle.global_bbox_max.y
            )
            for x in range(max(0, minx - 1), min(self.grid.info.width, maxx + 1)):
                for y in range(max(0, miny - 1), min(self.grid.info.height, maxy + 1)):
                    self.active_cells[x + y * self.grid.info.width] = False

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
                    curVal //= 2
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
