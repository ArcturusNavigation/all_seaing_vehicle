#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from all_seaing_interfaces.action import Waypoint
from geometry_msgs.msg import PointStamped


class WaypointSender(Node):

    def __init__(self):
        super().__init__("rviz_waypoint_sender")
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

    def point_callback(self, msg: PointStamped):
        goal_msg = Waypoint.Goal()
        goal_msg.xy_threshold = self.xy_threshold
        goal_msg.theta_threshold = self.theta_threshold
        goal_msg.x = msg.point.x
        goal_msg.y = msg.point.y
        goal_msg.ignore_theta = True

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
