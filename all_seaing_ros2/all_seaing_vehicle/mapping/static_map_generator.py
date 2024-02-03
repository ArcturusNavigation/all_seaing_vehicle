#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# Map resolution (meters per grid size)
MAP_RESOLUTION = 0.25

# Map dimensions
MAP_GRID_WIDTH = 600
MAP_GRID_HEIGHT = 400
MAP_METERS_WIDTH = MAP_GRID_WIDTH * MAP_RESOLUTION
MAP_METERS_HEIGHT = MAP_GRID_HEIGHT * MAP_RESOLUTION

# Map origin
MAP_ORIGIN_X = -MAP_METERS_WIDTH / 4
MAP_ORIGIN_Y = -MAP_METERS_HEIGHT / 6


class StaticMapGenerator(Node):

    def __init__(self):
        super().__init__("static_map_generator")
        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10),
        )
        self.timer = self.create_timer(
            timer_period_sec=1.0, callback=self.timer_callback
        )
        self.map_data = [0 for _ in range(MAP_GRID_WIDTH * MAP_GRID_HEIGHT)]

    def timer_callback(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.info.width = MAP_GRID_WIDTH
        grid.info.height = MAP_GRID_HEIGHT
        grid.info.resolution = MAP_RESOLUTION
        grid.info.origin.position.x = MAP_ORIGIN_X
        grid.info.origin.position.y = MAP_ORIGIN_Y
        grid.data = self.map_data
        self.publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
