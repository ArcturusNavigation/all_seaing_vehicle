#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
import random  # For random obstacle placement

class WayPointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseArray, 'waypoints', 10)
        timer_period = 5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # # Define map properties
        # self.map_width = 10  # In cells
        # self.map_height = 10  # In cells
        # self.map_resolution = 0.1  # In meters/cell
        # self.origin_position = [0.0, 0.0, 0.0]  # [x, y, theta]

        # # Initialize the grid (e.g., all cells as free space initially)
        # self.grid_data = np.zeros((self.map_width * self.map_height), dtype=int)  # Free space = 0

        # # Randomly populate some cells with obstacles
        # self.populate_obstacles(obstacle_count=20)  # Place 20 obstacles randomly

    def populate_obstacles(self, obstacle_count):
        """
        Randomly place obstacles in the grid. Obstacles are marked with value 100.
        """
        for _ in range(obstacle_count):
            random_index = random.randint(0, self.map_width * self.map_height - 1)
            self.grid_data[random_index] = 100  # Mark cell as an obstacle (occupied)

    def timer_callback(self):
        # Create OccupancyGrid message
        msg = PoseArray()

        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'waypoint'  # Frame of the map

        # Generate random poses
        # num_poses = random.randint(3, 10)  # Random number of poses
        num_poses = 2
        for _ in range(num_poses):
            pose = Pose()

            # Randomize position
            pose.position.x = float(random.uniform(0, 100))
            pose.position.y = float(random.randint(0, 100))

            # Add to pose array
            msg.poses.append(pose)

        # Publish the pose array
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Waypoints posearray with random poses')

def main(args=None):
    rclpy.init(args=args)

    waypoint_publisher = WayPointPublisher()

    rclpy.spin(waypoint_publisher)

    waypoint_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
