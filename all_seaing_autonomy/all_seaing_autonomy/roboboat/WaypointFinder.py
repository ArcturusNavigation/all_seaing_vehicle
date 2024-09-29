#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from geometry_msgs.msg import Point, Pose, PoseArray
from all_seaing_interfaces.msg import BuoyPair, BuoyPairArray, Waypoint, WaypointArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import math

class WaypointFinder(Node):
    def __init__(self):
        super().__init__("waypoint_finder")
        self.map_sub = self.create_subscription(
            ObstacleMap, "/obstacle_map/labeled", self.map_cb, 10
        )
        self.odometry_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odometry_cb, 10)
        self.buoy_pair_pub = self.create_publisher(BuoyPairArray, "buoy_pairs", 10)
        self.waypoint_pub = self.create_publisher(WaypointArray, "waypoints", 10)

        self.robot_pos = (0,0)
        self.safe_margin = 0

        self.first_map = True

    def norm_squared(self, vec, ref=(0, 0)):
        return vec[0] ** 2 + vec[1] ** 2

    def get_closest_to(self, source, points):
        return min(points, key=lambda point: self.norm_squared(source, point))

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)
    
    def midpoint_pair(self, pair):
        return self.midpoint(pair[0], pair[1])
    
    def split_buoys(self, obstacles):
        green_bouy_points = []
        red_bouy_points = []
        for obstacle in obstacles:
            if obstacle.label == 2:
                green_bouy_points.append(obstacle)
            elif obstacle.label == 1:
                red_bouy_points.append(obstacle)
        return green_bouy_points, red_bouy_points
    
    def obs_to_pos(self, obs):
        return [(ob.global_point.point.x, ob.global_point.point.y) for ob in obs]
    
    def obs_to_pos_label(self, obs):
        return [(ob.global_point.point.x, ob.global_point.point.y, ob.label) for ob in obs]

    def setup_buoys(self):
        self.get_logger().info("Setting up starting buoys!")
        self.get_logger().info("obstacles: "+str(self.obs_to_pos_label(self.obstacles)))
        green_init, red_init = self.split_buoys(self.obstacles)
        obstacles_in_front = lambda obs: [(ob.global_point.point.x, ob.global_point.point.y) for ob in obs if ob.local_point.point.x > 0]
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().info("initial red buoys: "+str(red_buoys)+", green buoys: "+str(green_buoys))
        if(len(red_buoys)==0 or len(green_buoys)==0):
            self.get_logger().warning("No starting buoy pairs!")
            return False
        self.starting_buoys = (self.get_closest_to((0,0),red_buoys), self.get_closest_to((0,0),green_buoys))
        self.pair_to = self.starting_buoys
        self.get_logger().info("starting buoys: "+str(self.starting_buoys))
        return True
    
    def ccw(self, a, b, c):
        """Check if the points a, b, c are counterclockwise, in this order, and return True, otherwise return False"""
        # literally just shoelace formula
        area = a[0]*b[1] + b[0]*c[1] + c[0]*a[1] - a[1]*b[0] - b[1]*c[0] - c[1]*a[0]
        if area > 0:
            return True
        else:
            return False
    
    def filter_front_buoys(self, pair, buoys):
        # (red, green)
        return [buoy for buoy in buoys if self.ccw(pair[0], pair[1], buoy)]

    def next_pair(self, prev_pair, red, green):
        """Find the next buoy pair (red left, green right) from the previous pair,
        by checking the closes one to the middle of the previous buoy pair that's in front of the pair"""
        return (self.get_closest_to(self.midpoint_pair(prev_pair), self.filter_front_buoys(prev_pair, red)), self.get_closest_to(self.midpoint(prev_pair), self.filter_front_buoys(prev_pair, green)))
    
    def pair_to_pose(self, pair):
        return Pose(position = Point(x = pair[0], y = pair[1]))
    
    def generate_waypoints(self):
        self.get_logger().info("obstacles: "+str(self.obs_to_pos_label(self.obstacles)))
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        green_buoys = self.obs_to_pos(green_buoys)
        red_buoys = self.obs_to_pos(red_buoys)
        self.get_logger().info("red buoys: "+str(red_buoys)+", green buoys: "+str(green_buoys))
        # RED BUOYS LEFT, GREEN RIGHT

        if len(green_buoys) == 0 or len(red_buoys) == 0:
            self.get_logger().warning("No buoy pairs!")
            return
        
        # TODO: Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair
        
        # Check if we passed that pair of buoys, then move on to the next one
        if self.ccw(self.pair_to[0],self.pair_to[1], self.robot_pos):
            try:
                self.pair_to = self.next_pair(self.pair_to, red_buoys, green_buoys)
            except:
                self.get_logger().warning("No next buoy pair to go to!")

        buoy_pairs = [self.pair_to]
        waypoints = [self.midpoint_pair(self.pair_to)]

        # will terminate if we run out of either of the points
        while True:
            last_waypoint = self.midpoint_pair(buoy_pairs[-1])
            try:
                buoy_pairs.append(self.next_pair(last_waypoint))
                waypoints.append(self.midpoint(buoy_pairs[-1]))
            except:
                break
        
        waypoint_arr = WaypointArray(waypoints = [Waypoint(point = self.pair_to_pose(wpt), radius = math.sqrt(self.norm_squared(pair[0], pair[1])) - self.safe_margin) for wpt, pair in zip(waypoints, buoy_pairs)])
        buoy_pairs = BuoyPairArray(pairs = [BuoyPair(left = self.pair_to_pose(pair[0]), right = self.pair_to_pose(pair[1]), waypoint = waypoint) for pair, waypoint in zip(buoy_pairs, waypoint_arr.waypoints)])
        self.buoy_pair_pub.publish(buoy_pairs)
        self.waypoint_pub.publish(waypoint_arr)
        self.get_logger().info("buoy pairs: "+str(buoy_pairs)+", waypoints: "+str(waypoint_arr))

    def map_cb(self, msg):
        self.obstacles = msg.obstacles
        self.get_logger().info("obstacles: "+str(self.obs_to_pos_label(self.obstacles)))
        
        success = False
        if self.first_map:
            success = self.setup_buoys()
            self.first_map = not success
        else:
            success = True
        if success:
            self.generate_waypoints()
        
    def odometry_cb(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = WaypointFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()