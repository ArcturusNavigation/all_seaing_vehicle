import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np  # For easier manipulation of grid data
import random  # For random obstacle placement

class MapPublisher(Node):

    def __init__(self):
        super().__init__('map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Define map properties
        self.map_width = 10  # In cells
        self.map_height = 10  # In cells
        self.map_resolution = 0.1  # In meters/cell
        self.origin_position = [0.0, 0.0, 0.0]  # [x, y, theta]

        # Initialize the grid (e.g., all cells as free space initially)
        self.grid_data = np.zeros((self.map_width * self.map_height), dtype=int)  # Free space = 0

        # Randomly populate some cells with obstacles
        self.populate_obstacles(obstacle_count=20)  # Place 20 obstacles randomly

    def populate_obstacles(self, obstacle_count):
        """
        Randomly place obstacles in the grid. Obstacles are marked with value 100.
        """
        for _ in range(obstacle_count):
            random_index = random.randint(0, self.map_width * self.map_height - 1)
            self.grid_data[random_index] = 100  # Mark cell as an obstacle (occupied)

    def timer_callback(self):
        # Create OccupancyGrid message
        msg = OccupancyGrid()

        # Populate the header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Frame of the map

        # Populate the MapMetaData
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution  # Meters per cell
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        # Set the origin (Pose of the bottom-left corner of the map in the frame_id)
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_position[0]
        msg.info.origin.position.y = self.origin_position[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # No rotation, quaternion identity

        # Populate the map data (row-major order)
        msg.data = list(self.grid_data)  # Flatten and convert to a list

        # Publish the occupancy grid
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: OccupancyGrid map with random obstacles')

def main(args=None):
    rclpy.init(args=args)

    map_publisher = MapPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
