#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_interfaces.action import Waypoint
from all_seaing_interfaces.msg import ControlOption
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


class ControllerServer(Node):
    def __init__(self):
        super().__init__("controller_server")

        self.group = MutuallyExclusiveCallbackGroup()

        self.timer_period = (
            self.declare_parameter("timer_period", 1 / 60)
            .get_parameter_value()
            .double_value
        )
        self.global_frame_id = (
            self.declare_parameter("global_frame_id", "odom")
            .get_parameter_value()
            .string_value
        )

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            callback_group=self.group,
            execute_callback=self.waypoint_callback,
            cancel_callback=self.cancel_callback,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)
        self.marker_pub = self.create_publisher(Marker, "control_marker", 10)

        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0
        self.proc_count = 0
        self.marker_ns = "control"

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

    def send_waypoint(self, x, y, angular):
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.linear_control_mode = ControlOption.WORLD_POSITION
        control_msg.angular_control_mode = ControlOption.WORLD_POSITION
        control_msg.x = x
        control_msg.y = y
        control_msg.angular = angular
        self.control_pub.publish(control_msg)

    def start_process(self, msg=None):
        self.proc_count += 1
        while self.proc_count >= 2:
            time.sleep(self.timer_period)
        if msg is not None:
            time.sleep(self.timer_period)
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
        marker_msg.ns = self.marker_ns
        marker_msg.action = Marker.DELETEALL
        self.marker_pub.publish(marker_msg)

    def visualize_waypoint(self, x, y):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = self.marker_ns
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

    def waypoint_callback(self, goal_handle):
        self.start_process("Waypoint following started!")

        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = (
            math.atan2(goal_y - self.nav_y, goal_x - self.nav_x)
            if goal_handle.request.ignore_theta
            else goal_handle.request.theta
        )
        self.visualize_waypoint(goal_x, goal_y)

        feedback_msg = Waypoint.Feedback()
        while (
            abs(goal_x - self.nav_x) > xy_threshold
            or abs(goal_y - self.nav_y) > xy_threshold
            or abs(goal_theta - self.heading) > math.radians(theta_threshold)
        ):
            if self.proc_count >= 2:
                self.end_process("Waypoint following aborted!")
                goal_handle.abort()
                return Waypoint.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Waypoint following canceled!")
                goal_handle.canceled()
                return Waypoint.Result()

            self.send_waypoint(goal_x, goal_y, goal_theta)
            feedback_msg.x = self.nav_x
            feedback_msg.y = self.nav_y
            feedback_msg.theta = self.heading
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(self.timer_period)

        self.end_process("Waypoint following completed!")
        goal_handle.succeed()
        return Waypoint.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
