#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from all_seaing_interfaces.action import Waypoint
from geometry_msgs.msg import PointStamped


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")
        self.xy_threshold = (
            self.declare_parameter("xy_threshold", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.theta_threshold = (
            self.declare_parameter("theta_threshold", 5.0)
            .get_parameter_value()
            .double_value
        )

        self.action_client = ActionClient(self, Waypoint, "waypoint")
        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )

        self.goal_handle = None

    def point_callback(self, msg: PointStamped):
        if self.goal_handle is not None and self.goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.cancel_goal_future = self.goal_handle.cancel_goal_async()

        goal_msg = Waypoint.Goal()
        goal_msg.xy_threshold = self.xy_threshold
        goal_msg.theta_threshold = self.theta_threshold
        goal_msg.x = msg.point.x
        goal_msg.y = msg.point.y
        goal_msg.ignore_theta = True

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Waypoint following finished?: {result.is_finished}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
