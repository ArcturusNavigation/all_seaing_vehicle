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
from a_star import *

TIMER_PERIOD = 1 / 60
MARKER_NS = "control"

class NavigationServer(Node):
    def __init__(self):
        super().__init__("navigation_server")

        #--------------- PARAMETERS ---------------#

        self.global_frame_id = self.declare_parameter(
            "global_frame_id", "odom").get_parameter_value().string_value
        Kpid_x = self.declare_parameter(
            "Kpid_x", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        Kpid_y = self.declare_parameter(
            "Kpid_y", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        Kpid_theta= self.declare_parameter(
            "Kpid_theta", [1.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.max_vel = self.declare_parameter(
            "max_vel", [4.0, 2.0, 1.0]).get_parameter_value().double_array_value

        #--------------- SUBSCRIBERS, PUBLISHERS, AND SERVERS ---------------#
        # goal_sub and path_sub removed
        self.group = MutuallyExclusiveCallbackGroup()

        self.map_topic = "map"  # OccupancyGrid

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            execute_callback=self.waypoint_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", map_cb, 10, callback_group=self.group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )

        self.pose_array_pub = self.create_publisher(PoseArray, "path_planning", 10)
        self.path_pub = self.create_publisher(Path, "a_star_path", 10)

        self.map_grid = None
        self.map_info = None  # Resolution of the map (m/cell)
        self.target = None
        self.cutoff = 50  # Threshold for obstacle cells
        self.waypoints = None
        self.failed_runs = 0
        self.completed_runs = 0

        self.get_logger().debug("Initialized A* Path Planner")

        # Publishers for path and PoseArray
        self.pose_array_pub = self.create_publisher(PoseArray, "path_planning", 10)
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

        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0
        self.proc_count = 0
        self.prev_update_time = self.get_clock().now()

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

        self.path_pub.publish

        self.get_logger().debug("Published A* Path")

    def odom_callback(self, msg: Odometry):
        self.nav_x = msg.pose.pose.position.x
        self.nav_y = msg.pose.pose.position.y
        _, _, self.heading = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

    def start_process(self, msg=None):
        self.proc_count += 1
        while self.proc_count >= 2:
            time.sleep(TIMER_PERIOD)
        if msg is not None:
            time.sleep(TIMER_PERIOD)
            self.get_logger().info(msg)

    def end_process(self, msg=None):
        self.delete_all_marker()
        self.proc_count -= 1
        if msg is not None:
            self.get_logger().info(msg)

    def delete_all_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = MARKER_NS
        marker_msg.action = Marker.DELETEALL
        self.marker_pub.publish(marker_msg)

    def visualize_waypoint(self, x, y):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = MARKER_NS
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 2.0
        marker_msg.scale.x = 0.4
        marker_msg.scale.y = 0.4
        marker_msg.scale.z = 8.0
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 1.0
        self.marker_pub.publish(marker_msg)

    def cancel_callback(self, cancel_request):
        return CancelResponse.ACCEPT

    def reset_pid(self):
        self.prev_update_time = self.get_clock().now()
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()

    def set_pid_setpoints(self, x, y, theta):
        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(theta)

    def update_pid(self):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.x_pid.update(self.nav_x, dt)
        self.y_pid.update(self.nav_y, dt)
        self.theta_pid.update(self.heading, dt)
        self.prev_update_time = self.get_clock().now()

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel

        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel

    def control_loop(self):
        self.update_pid()
        x_output = self.x_pid.get_effort()
        y_output = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output * math.cos(self.heading) + y_output * math.sin(self.heading)
        y_vel = y_output * math.cos(self.heading) - x_output * math.sin(self.heading)

        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)


    def waypoint_callback(self, goal_handle):
        self.start_process("Waypoint following started!")

        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y

        if goal_handle.request.ignore_theta:
            goal_theta = math.atan2(goal_y - self.nav_y, goal_x - self.nav_x)
        else:
            goal_theta = goal_handle.request.theta
        self.visualize_waypoint(goal_x, goal_y)

        #Generate a path using A* from the current position to the goal

        #TODO: call path correctly and follows each waypoint correctly
        start_pose = Pose()
        start_pose.position.x = self.nav_x
        start_pose.position.y = self.nav_y

        goal_pose = Pose()
        goal_pose.position.x = goal_x
        goal_pose.position.y = goal_y

        path = plan_path(self, start_pose, goal_pose)

        self.get_logger().info("Before publishing nav_path")

        if path is not None and len(path)>0:
            self.publish_nav_path(path)

        # Return a list of waypoitns the robot should follow
        if path is None or len(path) == 0:
            self.end_process("Waypoint following aborted!")
            goal_handle.abort()
            return Waypoint.Result()

        self.reset_pid()

        for waypoint in path:
            waypoint_x, waypoint_y = waypoint
            self.set_pid_setpoints(waypoint_x, waypoint_y, goal_theta)

            while (not self.x_pid.is_done(self.nav_x, xy_threshold) or
               not self.y_pid.is_done(self.nav_y, xy_threshold) or
               not self.theta_pid.is_done(self.heading, math.radians(theta_threshold))):

                if self.proc_count >= 2:
                    self.end_process("Waypoint following aborted!")
                    goal_handle.abort()
                    return Waypoint.Result()

                if goal_handle.is_cancel_requested:
                    self.end_process("Waypoint following canceled!")
                    goal_handle.canceled()
                    return Waypoint.Result()

            self.control_loop()
            time.sleep(TIMER_PERIOD)

        self.end_process("Waypoint following completed!")
        goal_handle.succeed()
        return Waypoint.Result(is_finished=True)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    node.get_logger().info("Navigation server node initialized.")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
