#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from all_seaing_interfaces.msg import ObstacleMap
from nav_msgs.msg import OccupancyGrid


class ObstacleMapper(Node):

    def __init__(self):
        super().__init__("obstacle_map")
        self.resolution = (
            self.declare_parameter("grid_resolution", 1.0) # TODO find actual resolution
            .get_parameter_value()
            .double_value
        )
        self.map_width = (
            self.declare_parameter("map_cell_width", 100) # TODO determine size
            .get_parameter_value()
            .int_value
        )
        self.map_height = (
            self.declare_parameter("map_cell_height", 100) # TODO determine size
            .get_parameter_value()
            .int_value
        )
        self.map_sub = self.create_subscription(
            ObstacleMap, "labeled_map", self.map_callback, 10 # TODO Find actual topic
        )
        self.map_pub = self.create_publisher(
            OccupancyGrid, "grid_map", 10 
        )

    def map_callback(self, msg: ObstacleMap):
        real_world_size = self.resolution * self.size
        offset = 0 # real_world_size / 2
        
        grid = [[0] * self.size for _ in range(self.size)]

        for obstacle in msg.obstacles:
            # x = obstacle.local_point.point.x
            # y = obstacle.local_point.point.y
            x = obstacle.global_point.point.x
            y = obstacle.global_point.point.y
            
            # convert to grid coords
            x += offset
            y += offset
            x //= self.resolution
            y //= self.resolution
            grid[x][y] = 1000

        # generate output message
        output_msg = OccupancyGrid()
        output_msg.header = msg.header

        output_msg.info = MapMetaData()
        output_msg.info.resolution = self.resolution
        output_msg.info.width = self.map_width
        output_msg.info.height = self.map_height

        output_msg.data = [x for x in row for row in grid]
        self.map_pub.publish(output_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
