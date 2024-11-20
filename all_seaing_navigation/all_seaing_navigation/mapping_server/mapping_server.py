#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from scipy.spatial import ConvexHull

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from all_seaing_interfaces.msg import ObstacleMap


class MappingServer(Node):
    """ Inputs obstacle convex hulls(labeled_map) and outputs global map (Occupancy_grid) """

    def __init__(self):
        super().__init__("mapping_server")

        self.labeled_map_topic = "labeled_map"

        # Subscriber to labeled_map
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.labeled_map_sub = self.create_subscription(
            ObstacleMap, self.labeled_map_topic, self.hulls_update_cb, 10,callback_group=self.group1)

        # Publishers for path and PoseArray
        self.grid_pub = self.create_publisher(OccupancyGrid, "occupancy_grid", 20, callback_group=self.group2)
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.group2)
        self.iterations = 0

        # Local variables
        self.labeled_map = ObstacleMap()

        # Actual location of competition (Nathan Benderson Park) has an 
        #     interior square area of around 117m * 117m. We use a 200m * 200m square.
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'

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
        self.ship_pos = (10,42)
        self.lidar_rad = 10

        # Active cells in row major order            
        self.active_cells = [False] * self.grid.info.width * self.grid.info.height
        self.get_logger().info("Initialized Mapping Server")

    def ccw(self,A,B,C):
        v1,v2 = (B[0]-A[0], B[1]-A[1]), (C[0]-B[0], C[1]-B[1])
        return v1[0]*v2[1] - v1[1]*v2[0]
    def dot(self,A,B):
        return A[0]*B[0]+A[1]*B[1]
    def intersect(self,A,B,C,D):
        if self.ccw(A,B,C) == 0 or self.ccw(A,B,D) == 0:
            vB = (B[0]-A[0], B[1]-A[1])
            aC = (C[0]-A[0], C[1]-A[1])
            bC = (C[0]-B[0], C[1]-B[1])
            aD = (D[0]-A[0], D[1]-A[1])
            bD = (D[0]-B[0], D[1]-B[1])
            return self.dot(aC,aC) + self.dot(bC, bC) <= self.dot(vB, vB) or self.dot(aD,aD) + self.dot(bD, bD) <= self.dot(vB, vB)
        return self.ccw(A,C,D) * self.ccw(B,C,D) < 0 and self.ccw(A,B,C) * self.ccw(A,B,D) < 0

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
        Find the current active cells that the sensors can see, and mark them as active
        Mark cells inactive if behind obstacles
        """
        all_modified_hulls = []
        all_mpolys = []
        for obstacle in self.labeled_map.obstacles:
            gchull = obstacle.global_chull.polygon.points
            m_polygon = [(point.x, point.y) for point in gchull]
            # m_polygon.append(self.ship_pos)
            all_mpolys.append(m_polygon)
            all_modified_hulls.append(ConvexHull(m_polygon))

        #test if cells are active
        #O(W * H * |hulls|) (very slow)
        self.get_logger().info("Started finding active cells")
        for x in range(0, self.grid.info.width):
            for y in range(0, self.grid.info.height):
                worldx, worldy = self.grid_to_world(x,y)
                rad = (worldx-self.ship_pos[0])**2 + (worldy-self.ship_pos[1])**2
                if (worldx-self.ship_pos[0])**2 + (worldy-self.ship_pos[1])**2 > self.lidar_rad**2:
                    self.active_cells[x + y * self.grid.info.width] = False
                    continue
                self.active_cells[x + y * self.grid.info.width] = True
                if len(self.labeled_map.obstacles) == 0:
                    continue
                for mhull, mpoly in zip(all_modified_hulls, all_mpolys):
                    # Check if cell is behind convex hull
                    ctr = 0
                    for line_indices in mhull.simplices: #indices of points that form a line
                        ctr += (self.intersect(mpoly[line_indices[0]], mpoly[line_indices[1]],
                                                ((worldx),(worldy)), (self.ship_pos[0], self.ship_pos[1])))
                    if ctr == 2:
                        continue
                    if ctr > 2:
                        self.active_cells[x + y * self.grid.info.width] = False
                        break

        self.get_logger().info("Found active cells")
        self.modify_probability()

    def modify_probability(self):
        """Decay or increase probability of obstacle in active cells based on sensor observations"""
        for x in range(0, self.grid.info.width):
            for y in range(0, self.grid.info.height):
                if self.active_cells[x + y*self.grid.info.width]:
                    self.grid.data[x+y*self.grid.info.width] = 0
                else:
                    self.grid.data[x+y*self.grid.info.width] = 100
    
    def timer_callback(self):
        self.iterations += 1
        self.publish_grid()


    
    def hulls_update_cb(self, msg):
        self.labeled_map = msg
        self.get_logger().info("Mapping server: Intialized obstacles" if self.labeled_map is None else "Mapping server: Updated obstacles")
        self.find_active_cells()

    def publish_grid(self):
        """Publish grid as nav_msgs/Grid"""
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.grid)
        self.get_logger().info("Mapping Server: Published Occupancy Grid")

def main(args=None):
    rclpy.init(args=args)
    node = MappingServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
