#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from all_seaing_interfaces.msg import ControlMessage
from geometry_msgs.msg import PointStamped


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")
        self.subscription = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.point_callback,
            10,
        )
        self.timer = self.create_timer(1/60, self.timer_callback)
        self.publisher = self.create_publisher(ControlMessage, "control_options", 10)
        self.control_msg = ControlMessage()
        self.control_msg.angular = 0.0
        self.control_msg.linear_control_mode = ControlMessage.WORLD_POSITION
        self.control_msg.angular_control_mode = ControlMessage.WORLD_POSITION

    def point_callback(self, msg: PointStamped):
        self.control_msg.x = msg.point.x
        self.control_msg.y = msg.point.y
    
    def timer_callback(self):
        self.publisher.publish(self.control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
