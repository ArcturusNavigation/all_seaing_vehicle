#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid

class BuoyMapper(Node):

    def __init__(self):
        super().__init__("buoy_mapper")
        self.subscription = self.create_subscription(
            PoseArray,
            "/obstacles",
            self.callback,
            10
        ) 
        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/map",
            QoSProfile(
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )

    def callback(self, msg):
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.info.width = 400
        grid.info.height = 400 # THIS IS THE NUMBER OF GRIDS
        grid.info.resolution = 0.1
        grid.info.origin.position.x = -20.0 # THIS IS IN METERS
        grid.info.origin.position.y = -20.0
        grid.data = [0 for _ in range(400 * 400)]
        self.publisher.publish(grid)  

def main(args=None):
    rclpy.init(args=args)
    node = BuoyMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
