#!/usr/bin/env python3
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from all_seaing_interfaces.action import Waypoint, FollowPath
from geometry_msgs.msg import PointStamped


class WaypointSender(Node):

    def __init__(self):
        super().__init__("rviz_waypoint_sender")

        self.declare_parameter("xy_threshold", 2.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 5)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )

    def send_waypoint(self, msg: PointStamped):
        goal_msg = Waypoint.Goal()
        goal_msg.x = msg.point.x
        goal_msg.y = msg.point.y
        goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
        goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
        goal_msg.ignore_theta = True
        goal_msg.is_stationary = True
        self.waypoint_client.wait_for_server()
        self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)

    def send_path(self, msg: PointStamped):
        goal_msg = FollowPath.Goal()
        goal_msg.planner = self.get_parameter("planner").value
        goal_msg.x = msg.point.x
        goal_msg.y = msg.point.y
        goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
        goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
        goal_msg.goal_tol = self.get_parameter("goal_tol").value
        goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
        goal_msg.choose_every = self.get_parameter("choose_every").value
        goal_msg.is_stationary = True
        self.follow_path_client.wait_for_server()
        self.send_goal_future = self.follow_path_client.send_goal_async(goal_msg)

    def point_callback(self, msg: PointStamped):
        use_waypoint_client = self.get_parameter("use_waypoint_client").value

        if use_waypoint_client:
            self.send_waypoint(msg)
        else:
            self.send_path(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
