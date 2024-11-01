#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from all_seaing_interfaces.msg import ObstacleMap


class PathPlan(Node):
    """ Inputs obstacle convex hulls(labeled_map) and outputs global map (Occupancy_grid) """

    def __init__(self):
        super().__init__("mapping_server")

        self.labeled_map_topic = "labeled_map"

        # Subscriber to labeled_map
        self.labeled_map_sub = self.create_subscription(
            ObstacleMap, self.labeled_map_topic, self.map_cb, 10)

        # Publishers for path and PoseArray
        self.grid_pub = self.create_publisher(OccupancyGrid, "occupancy_grid", 10)

        # Local variables
        self.labeled_map = ObstacleMap()

        # Actual location of competition (Nathan Benderson Park) has an 
        # interior square area of around 117m * 117m. We use a 200m * 200m square.
        # Occupancy Grid indexing: index = X+Y*width
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'grid'
        self.grid.info.width = 2000
        self.grid.info.height = 2000
        self.grid.info.resolution = 0.1

        # Position of ship relative to global origin
        self.ship_pos = (0,0)


        # Active cells in row major order            
        self.active_cells = [[False] * self.grid.info.width * self.grid.info.height]
        self.get_logger().debug("Initialized Mapping Server")

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        gx = int((x - origin.x) / resolution)
        gy = int((y - origin.y) / resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates back to world coordinates."""
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        x = gx * resolution + origin.x
        y = gy * resolution + origin.y
        return x, y

    def find_active_cells(self):
        """Find the current active cells that the sensors can see, and mark them as active"""
        
        pass

    def modify_probability(self):
        """Decay or increase probability of obstacle in active cells based on sensor observations"""
        pass
    
    def hulls_update_cb(self, msg):
        self.labeled_map = msg
        self.get_logger().debug("Mapping server: Intialized obstacles" if self.labeled_map is None else "Mapping server: Updated obstacles")
    
    def publish_grid(self):
        """Publish grid as nav_msgs/Grid"""
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid)
        self.get_logger().debug("Mapping Server: Published Occupancy Grid")


    # def pose_to_string(self, pos):
    #     return f"{{{pos.position.x}, {pos.position.y}, {pos.position.z}}}"

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
