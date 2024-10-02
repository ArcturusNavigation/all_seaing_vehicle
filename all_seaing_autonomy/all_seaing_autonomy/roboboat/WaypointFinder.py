#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3, Quaternion
from all_seaing_interfaces.msg import BuoyPair, BuoyPairArray, Waypoint, WaypointArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
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
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, "waypoint_markers", 10)

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

    def buoy_pairs_to_markers(self, buoy_pairs):
        marker_array = MarkerArray()
        i = 0
        for buoy_pair in buoy_pairs.pairs:
            marker_array.markers.append(Marker(type=Marker.ARROW, pose=buoy_pair.waypoint.point, header = Header(frame_id="odom"), scale=Vector3(x=1.0,y=0.05,z=0.05), color=ColorRGBA(a=1.0), id=3*i))
            marker_array.markers.append(Marker(type=Marker.SPHERE, pose=buoy_pair.left, header = Header(frame_id="odom"), scale=Vector3(x=1.0,y=1.0,z=1.0), color=ColorRGBA(r = 1.0, a=1.0), id=3*i+1))
            marker_array.markers.append(Marker(type=Marker.SPHERE, pose=buoy_pair.right, header = Header(frame_id="odom"), scale=Vector3(x=1.0,y=1.0,z=1.0), color=ColorRGBA(g = 1.0, a=1.0), id=3*i+2))
        return marker_array

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
        
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
    
    def filter_front_buoys(self, pair, buoys):
        # (red, green)
        return [buoy for buoy in buoys if self.ccw(pair[0], pair[1], buoy)]

    def next_pair(self, prev_pair, red, green):
        """Find the next buoy pair (red left, green right) from the previous pair,
        by checking the closes one to the middle of the previous buoy pair that's in front of the pair"""
        return (self.get_closest_to(self.midpoint_pair(prev_pair), self.filter_front_buoys(prev_pair, red)), self.get_closest_to(self.midpoint(prev_pair), self.filter_front_buoys(prev_pair, green)))
    
    def pair_to_pose(self, pair):
        return Pose(position = Point(x = pair[0], y = pair[1]))
    
    def pair_angle_to_pose(self, pair, angle):
        quat = self.quaternion_from_euler(0,0,angle)
        return Pose(position = Point(x = pair[0], y = pair[1]), orientation = Quaternion(x = quat[0], y = quat[2], z = quat[2], w = quat[3]))

    def generate_waypoints(self):
        self.get_logger().info("obstacles: "+str(self.obs_to_pos_label(self.obstacles)))
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        green_buoys = self.obs_to_pos(green_buoys)
        red_buoys = self.obs_to_pos(red_buoys)
        self.get_logger().info("red buoys: "+str(red_buoys)+", green buoys: "+str(green_buoys))
        # RED BUOYS LEFT, GREEN RIGHT

        # if len(green_buoys) == 0 or len(red_buoys) == 0:
        #     self.get_logger().warning("No buoy pairs (other than start)!")
        #     return
        
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
        
        waypoint_arr = WaypointArray(waypoints = [Waypoint(point = self.pair_angle_to_pose(wpt,-math.atan(pair[1][0]-pair[0][0])/(pair[1][1]-pair[0][1])), radius = math.sqrt(self.norm_squared(pair[0], pair[1])) - self.safe_margin) for wpt, pair in zip(waypoints, buoy_pairs)])
        buoy_pair_arr = BuoyPairArray(pairs = [BuoyPair(left = self.pair_to_pose(pair[0]), right = self.pair_to_pose(pair[1]), waypoint = waypoint) for pair, waypoint in zip(buoy_pairs, waypoint_arr.waypoints)])
        
        self.buoy_pair_pub.publish(buoy_pair_arr)
        self.waypoint_pub.publish(waypoint_arr)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers(buoy_pair_arr))
        self.get_logger().info("buoy pairs: "+str(buoy_pairs)+", waypoints: "+str(waypoints))

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