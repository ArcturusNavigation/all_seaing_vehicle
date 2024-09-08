#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_interfaces.action import Waypoint
from all_seaing_interfaces.msg import ControlMessage
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class ControllerServer(Node):
    def __init__(self):
        super().__init__("controller_server")

        self.group = MutuallyExclusiveCallbackGroup()

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            callback_group=self.group,
            execute_callback=self.waypoint_callback,
            cancel_callback=self.waypoint_cancel_callback,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )
        self.control_pub = self.create_publisher(ControlMessage, "control_options", 10)

        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0

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
        control_message = ControlMessage()
        control_message.priority = 1
        control_message.linear_control_mode = ControlMessage.WORLD_POSITION
        control_message.angular_control_mode = ControlMessage.WORLD_POSITION
        control_message.x = x
        control_message.y = y
        control_message.angular = angular
        self.control_pub.publish(control_message)

    def waypoint_cancel_callback(self, cancel_request):
        self.get_logger().info("Waypoint following canceled!")
        return CancelResponse.ACCEPT

    def waypoint_callback(self, goal_handle):
        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = goal_handle.request.theta
        ignore_theta = goal_handle.request.ignore_theta

        if ignore_theta:
            goal_theta = math.atan2(goal_y - self.nav_y, goal_x - self.nav_x)

        self.get_logger().info(
            f"Navigating to x: {goal_x:.4}, y: {goal_y:.4}, theta: {goal_theta:.4}"
        )

        feedback_msg = Waypoint.Feedback()
        while (
            abs(goal_x - self.nav_x) > xy_threshold
            or abs(goal_y - self.nav_y) > xy_threshold
            or abs(goal_theta - self.heading) > math.radians(theta_threshold)
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Waypoint.Result()

            self.send_waypoint(goal_x, goal_y, goal_theta)
            feedback_msg.x = self.nav_x
            feedback_msg.y = self.nav_y
            feedback_msg.theta = self.heading
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1 / 60)

        goal_handle.succeed()
        result = Waypoint.Result()
        result.is_finished = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
