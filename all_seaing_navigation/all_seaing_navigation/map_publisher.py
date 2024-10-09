#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import random

class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define map properties
        self.map_width = 10
        self.map_height = 10
        self.map_resolution = 0.1
        self.origin_position = [0.0, 0.0, 0.0]

        # Initialize grid (unknown = -1, free space = 0)
        self.grid_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Change to 2D array

        # Populate some cells with obstacles (occupied = 100) and free space (0)
        self.populate_obstacles_and_free_space(obstacle_count=20)

    def populate_obstacles_and_free_space(self, obstacle_count):
        """
        Randomly place obstacles and free space in the grid. Obstacles are marked with value 100, free space with 0.
        """
        total_cells = self.map_height * self.map_width

        # Randomly choose obstacle locations
        obstacle_indices = random.sample(range(total_cells), obstacle_count)
        for index in obstacle_indices:
            row = index // self.map_width
            col = index % self.map_width
            self.grid_data[row, col] = 100  # Mark cell as an obstacle (occupied)

        # Randomly assign free space (0) to the remaining unknown cells
        for row in range(self.map_height):
            for col in range(self.map_width):
                if self.grid_data[row, col] == -1:  # If the cell is unknown
                    self.grid_data[row, col] = random.choice([0, -1])  # Randomly make it free space or keep as unknown

    def timer_callback(self):
        # Create OccupancyGrid message
        msg = OccupancyGrid()

        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Populate the MapMetaData
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        # Set the origin (Pose of the bottom-left corner of the map in the frame_id)
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_position[0]
        msg.info.origin.position.y = self.origin_position[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Flatten the 2D array and ensure it is within the valid int8 range
        msg.data = self.grid_data.flatten().astype(np.int8).tolist()

        # Publish the occupancy grid
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: OccupancyGrid map with random obstacles and free space')

        # For visualization purposes, print the grid as a 2D array
        self.print_grid()

    def print_grid(self):
        """Print the grid as a 2D array for visualization."""
        grid_2d = self.grid_data  # This is already a 2D array
        for row in grid_2d:
            print(' '.join(f'{cell:3}' for cell in row))  # Format each cell for better visualization

def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
