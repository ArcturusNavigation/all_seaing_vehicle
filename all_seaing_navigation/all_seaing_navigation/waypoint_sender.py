#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from all_seaing_interfaces.msg import ControlMessage
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")
        self.goal_threshold = self.declare_parameter(
            "goal_threshold", 1.0).get_parameter_value().double_value

        self.is_running = False

        self.control_msg = ControlMessage()
        self.control_msg.priority = 0
        self.control_msg.state = ControlMessage.TELEOP
        self.control_msg.linear_control_mode = ControlMessage.WORLD_POSITION
        self.control_msg.angular_control_mode = ControlMessage.WORLD_POSITION
        self.control_msg.angular = 0.0

        self.nav_x = 0.0
        self.nav_y = 0.0

        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.odom_callback, 10
        )
        self.control_pub = self.create_publisher(
            ControlMessage, "control_options", 10
        )
        self.timer = self.create_timer(1/60, self.timer_callback)

    
    def point_callback(self, msg: PointStamped):
        self.control_msg.x = msg.point.x
        self.control_msg.y = msg.point.y
        self.is_running = True

    def odom_callback(self, msg: Odometry):
        self.nav_x = msg.pose.pose.position.x
        self.nav_y = msg.pose.pose.position.y
    
    def timer_callback(self):
        if self.is_running:
            if (abs(self.nav_x - self.control_msg.x) < self.goal_threshold and
                abs(self.nav_y - self.control_msg.y) < self.goal_threshold):
                self.is_running = False
            self.control_pub.publish(self.control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
