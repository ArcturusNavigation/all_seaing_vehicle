#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class StaticMapGenerator(Node):

    def __init__(self):
        super().__init__("static_map_generator")
        
        # Initialize parameters
        self.declare_parameter("map_resolution", 0.25)
        self.declare_parameter("grid_width", 600)
        self.declare_parameter("grid_height", 400)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.map_resolution = self.get_parameter("map_resolution").value
        self.grid_width = self.get_parameter("grid_width").value
        self.grid_height = self.get_parameter("grid_height").value
        self.origin_x = self.get_parameter("origin_x").value
        self.origin_y = self.get_parameter("origin_y").value

        # Publishers and subscribers
        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10),
        )
        self.timer = self.create_timer(
            timer_period_sec=1.0, callback=self.timer_callback
        )
        self.map_data = [0 for _ in range(self.grid_width * self.grid_height)]

    def timer_callback(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.info.width = self.grid_width
        grid.info.height = self.grid_height
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -self.origin_x
        grid.info.origin.position.y = -self.origin_y
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
