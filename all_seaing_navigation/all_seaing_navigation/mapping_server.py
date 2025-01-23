#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from all_seaing_interfaces.msg import ObstacleMap


class MappingServer(Node):
    """Inputs obstacle convex hulls(labeled_map) and outputs global map (Occupancy_grid)"""

    def __init__(self):
        super().__init__("mapping_server")

        # Subscriber to labeled_map
        self.obstalce_map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/raw", self.hulls_update_cb, 10
        )

        # Publishers for path and PoseArray
        self.grid_pub = self.create_publisher(OccupancyGrid, "map/global", 10)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Local variables
        self.obstalce_map = ObstacleMap()

        # Actual location of competition (Nathan Benderson Park) has an
        #     interior square area of around 117m * 117m. We use a 200m * 200m square.
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = "odom"

        self.grid.info = MapMetaData()
        self.grid.info.width = 2000
        self.grid.info.height = 2000
        self.grid.info.resolution = 0.1

        self.grid.info.origin = Pose()
        self.grid.info.origin.position.x = 0.0
        self.grid.info.origin.position.y = 0.0
        self.grid.info.origin.position.z = 0.0

        self.grid.data = [-1] * self.grid.info.width * self.grid.info.height

        # Position of ship relative to global origin
        # TO DO: receive and set the position of the ship somewhere
        self.ship_pos = (100, 410)
        self.lidar_rad = 200

        # Active cells in row major order
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
        for obstacle in self.obstalce_map.obstacles:
            minx, miny = self.world_to_grid(
                obstacle.global_bbox_min.x, obstacle.global_bbox_min.y
            )
            maxx, maxy = self.world_to_grid(
                obstacle.global_bbox_max.x, obstacle.global_bbox_max.y
            )
            for x in range(max(0, minx - 1), min(self.grid.info.width, maxx + 1) + 1):
                for y in range(
                    max(0, miny - 1), min(self.grid.info.height, maxy + 1) + 1
                ):
                    self.active_cells[x + y * self.grid.info.width] = True
                    self.grid.data[x + y * self.grid.info.width] = 100

        self.modify_probability()

        for obstacle in self.obstalce_map.obstacles:
            minx, miny = self.world_to_grid(
                obstacle.global_bbox_min.x, obstacle.global_bbox_min.y
            )
            maxx, maxy = self.world_to_grid(
                obstacle.global_bbox_max.x, obstacle.global_bbox_max.y
            )
            for x in range(max(0, minx - 1), min(self.grid.info.width, maxx + 1) + 1):
                for y in range(
                    max(0, miny - 1), min(self.grid.info.height, maxy + 1) + 1
                ):
                    self.active_cells[x + y * self.grid.info.width] = False

    def modify_probability(self):
        """Decay or increase probability of obstacle in active cells based on sensor observations"""
        for x in range(
            max(0, self.ship_pos[0] - self.lidar_rad),
            min(self.grid.info.width, self.ship_pos[0] + self.lidar_rad + 1),
        ):
            for y in range(
                max(0, self.ship_pos[1] - self.lidar_rad),
                min(self.grid.info.height, self.ship_pos[1] + self.lidar_rad + 1),
            ):
                if (x - self.ship_pos[0]) ** 2 + (
                    y - self.ship_pos[1]
                ) ** 2 > self.lidar_rad**2:
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
        self.publish_grid()

    def hulls_update_cb(self, msg):
        self.obstalce_map = msg
        self.ship_pos = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        self.find_active_cells()

    def publish_grid(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = MappingServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
