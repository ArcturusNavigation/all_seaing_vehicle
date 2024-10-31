#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_controller.pid_controller import CircularPID, PIDController
from all_seaing_interfaces.action import Waypoint
from all_seaing_interfaces.msg import ControlOption
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from a_star import *  # Ensure the A* algorithm is imported

TIMER_PERIOD = 1 / 60
MARKER_NS = "control"

class NavigationServer(Node):
    def __init__(self):
        super().__init__("navigation_server")

        #--------------- PARAMETERS ---------------#
        self.global_frame_id = self.declare_parameter("global_frame_id", "odom").get_parameter_value().string_value
        Kpid_x = self.declare_parameter("Kpid_x", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        Kpid_y = self.declare_parameter("Kpid_y", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        Kpid_theta = self.declare_parameter("Kpid_theta", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.max_vel = self.declare_parameter("max_vel", [4.0, 2.0, 1.0]).get_parameter_value().double_array_value

        #--------------- SUBSCRIBERS AND PUBLISHERS ---------------#
        self.group = MutuallyExclusiveCallbackGroup()
        self.waypoint_server = ActionServer(
            self, Waypoint, "waypoint", execute_callback=self.waypoint_callback,
            cancel_callback=self.cancel_callback, callback_group=self.group)

        self.odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.odom_callback, 10, callback_group=self.group)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_cb, 10)

        self.goal_sub = self.create_subscription(
            PoseArray, "clicked_point", self.waypoints_cb, 10)

        self.path_pub = self.create_publisher(Path, "a_star_path", 10)
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)
        self.marker_pub = self.create_publisher(Marker, "control_marker", 10)

        #--------------- PID CONTROLLERS ---------------#
        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])

        #--------------- MEMBER VARIABLES ---------------#
        self.map_grid = None
        self.map_info = None  # Resolution of the map (m/cell)
        self.target = None
        self.cutoff = 50  # Threshold for obstacle cells
        self.waypoints = None
        self.path = []
        self.current_target_index = 0
        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0
        self.proc_count = 0
        self.prev_update_time = self.get_clock().now()

        self.get_logger().info("Navigation Server with Path Planning initialized")

    #--------------- CALLBACK FUNCTIONS ---------------#
    def map_cb(self, msg: OccupancyGrid):
        """Callback to handle the map data"""
        self.map_grid = msg.data
        self.map_info = msg.info
        self.get_logger().info("Map received")

    def waypoints_cb(self, msg: PoseArray):
        """Callback to handle waypoints"""
        self.waypoints = [(pose.position.x, pose.position.y) for pose in msg.poses]
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints")

    def odom_callback(self, msg: Odometry):
        """Updates the current position and orientation"""
        self.nav_x = msg.pose.pose.position.x
        self.nav_y = msg.pose.pose.position.y
        _, _, self.heading = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def generate_path(self, start, goal):
        """Generates a path using the A* algorithm"""
        if self.map_grid is None or self.waypoints is None:
            self.get_logger().warn("Map or waypoints not available yet")
            return

        # Assuming `plan_path()` is a function from the imported A* module
        self.path = plan_path(start, goal, self.map_grid, self.map_info, self.cutoff)
        if self.path:
            self.publish_nav_path(self.path)
            self.get_logger().info(f"Path generated with {len(self.path)} waypoints")
        else:
            self.get_logger().warn("Path planning failed")

    def publish_nav_path(self, path):
        """Publish path as nav_msgs/Path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in path:
            wx, wy = self.grid_to_world(point[0], point[1])  # Convert grid to world coordinates
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().debug("Published A* Path")

    #--------------- ACTION SERVER CALLBACK ---------------#
    def waypoint_callback(self, goal_handle):
        self.start_process("Waypoint following started!")

        # Path generation logic (start and goal positions are set from the waypoints)
        if self.waypoints and len(self.waypoints) >= 2:
            start = (self.nav_x, self.nav_y)
            goal = self.waypoints[-1]  # Assuming the last point is the goal
            self.generate_path(start, goal)
        else:
            self.end_process("No valid waypoints for path generation")
            goal_handle.abort()
            return Waypoint.Result()

        # Continue with the rest of the waypoint following logic...

    # Additional methods from your original implementation go here (e.g., control_loop, set_pid_setpoints, etc.)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
