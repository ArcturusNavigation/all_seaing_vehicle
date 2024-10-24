#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import random

class MapPublisherTest(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 100)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define map properties
        self.map_width = 100
        self.map_height = 100
        self.map_resolution = 0.1
        self.origin_position = [0.0, 0.0, 0.0] # [-5.0, -5.0, 0.0]

        # Initialize grid (unknown = -1, free space = 0)
        self.grid_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Start with all unknown

        # Populate the grid with random values
        self.populate_grid_with_random_values()
        # self.populate_blobs()

    def populate_grid_with_random_values(self):
        """
        Populate the grid with random probabilities for free space, obstacles, and unknowns.
        Free space will be 0-99, obstacles will be 100, and unknown will be -1.
        """
        for row in range(self.map_height):
            for col in range(self.map_width):
                rand_val = random.randint(0, 100)
                if rand_val < 5:  # 5% chance of being an obstacle
                    self.grid_data[row, col] = 100  # Mark cell as an obstacle
                elif rand_val >= 5 and rand_val < 90:  # 85% chance of being free space
                    self.grid_data[row, col] = 0  # Mark cell as free space
                elif rand_val >= 90 and rand_val < 100:  # 10% chance of being unknown
                    self.grid_data[row, col] = -1  # Mark cell as unknown
                else:
                    # Introduce intermediate values for uncertainty
                    self.grid_data[row, col] = random.randint(1, 99)  # Mark cell with a value between 1 and 99
        

    def populate_blobs(self):
        """
        Populate the grid with blobs with random probabilities for free space, obstacles, and unknowns.
        Free space will be 0-99, obstacles will be 100, and unknown will be -1.
        """
        for row in range(self.map_height):
            for col in range(self.map_width):
                rand_val = random.randint(0, 1000)
                if rand_val >= 1:  # .1% chance of being an obstacle
                    continue
                #make blobs of radius 10
                rad = 10
                for dr in range(-rad, rad+1):
                    for dc in range(-rad, rad+1):
                        nr = row+dr
                        nc = col+dc
                        if dr*dr+dc*dc > rad**2:
                            continue
                        if nr<0 or nr >= 100 or nc <0 or nc >=100:
                            continue
                        self.grid_data[nr,nc] = 90
        for row in range(self.map_height):
            for col in range(self.map_width):
                if self.grid_data[row, col] != 90:
                    self.grid_data[row, col] = 0

    def timer_callback(self):
        # Modify the grid data before publishing
        # self.populate_grid_with_random_values()
        # self.populate_blobs()

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
        self.get_logger().info('Publishing: OccupancyGrid map with random obstacles')

        # For visualization purposes, print the grid as a 2D array
        self.print_grid()


    def print_grid(self):
        """Print the grid as a 2D array for visualization."""
        grid_2d = self.grid_data  # This is already a 2D array
        for row in grid_2d:
            print(' '.join(f'{cell:3}' for cell in row))  # Format each cell for better visualization

def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisherTest()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
