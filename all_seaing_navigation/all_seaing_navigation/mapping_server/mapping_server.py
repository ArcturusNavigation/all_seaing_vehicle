#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.spatial import ConvexHull
# from utils import GeometryPrimitives

from nav_msgs.msg import OccupancyGrid
from all_seaing_interfaces.msg import ObstacleMap


class PathPlan(Node):
    """ Inputs obstacle convex hulls(labeled_map) and outputs global map (Occupancy_grid) """

    def __init__(self):
        super().__init__("mapping_server")

        self.labeled_map_topic = "labeled_map"

        # Subscriber to labeled_map
        self.labeled_map_sub = self.create_subscription(
            ObstacleMap, self.labeled_map_topic, self.hulls_update_cb, 10)

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
        self.lidar_rad = 10


        # Active cells in row major order            
        self.active_cells = [[False] * self.grid.info.width * self.grid.info.height]
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
        if len(self.labeled_map.obstacles) == 0:
            for x in range(0, self.grid.width):
                for y in range(0, self.grid.height):
                    self.active_cells[x+y*self.grid.width] = True
                    return
        """
        Find the current active cells that the sensors can see, and mark them as active
        Mark cells inactive if behind obstacles
        """

        ob = self.labeled_map.obstacles[0]
        self.ship_pos = (ob.local_point.point.x-ob.global_point.point.x,
                    ob.local_point.point.y-ob.global_point.point.y)
        all_modified_hulls = []
        all_hulls = []
        for obstacle in self.labeled_map.obstacles:
            gchull = obstacle.global_chull.polygon.points
            polygon = [(point.x, point.y) for point in gchull]
            all_hulls.append(ConvexHull(polygon))
            polygon.append(self.ship_pos)
            modified_hull = ConvexHull(polygon)
            all_modified_hulls.append(modified_hull)

        #test if cells are active
        #O(W * H * |hulls|) (very slow)
        for x in range(0, self.grid.width):
            for y in range(0, self.grid.height):
                worldx, worldy = self.grid_to_world(x), self.grid_to_world(y)
                if (worldx-self.ship_pos[0])**2 + (worldy-self.ship_pos[1])**2 > self.lidar_rad**2:
                    self.active_cells[x + y * self.grid.width] = False
                    continue

                self.active_cells[x + y * self.grid.width] = True
                for hull,mhull in zip(all_hulls,all_modified_hulls):
                    continue_flag = False
                    for line_indices in hull.simplices:
                        if self.intersect(line_indices[0], line_indices[1],
                                         worldx-self.ship_pos[0], worldy-self.ship_pos[1]):
                            continue_flag = True
                            break
                    if continue_flag: 
                        continue

                    # Check if cell is behind convex hull
                    ctr = 0
                    for line_indices in mhull.simplices: #indices of points that form a line
                        ctr += (self.intersect(line_indices[0], line_indices[1],
                                                (worldx,worldy), (self.grid.info.width+0.00007,self.grid.info.height)))
                    if ctr%2 == 0:
                        self.active_cells[x + y * self.grid.width] = False
                        break

    def modify_probability(self):
        """Decay or increase probability of obstacle in active cells based on sensor observations"""
        for x in range(0, self.grid.width):
            for y in range(0, self.grid.height):
                self.grid[x+y*self.grid.width] = 100 if self.active_cells[x+y*self.grid.width] else 0

    
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
