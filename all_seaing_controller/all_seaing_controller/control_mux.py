#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlOption
from geometry_msgs.msg import Twist

TIMER_PERIOD = 1 / 2

class ControlMux(Node):
    def __init__(self):
        super().__init__("control_mux")

        self.control_sub = self.create_subscription(
            ControlOption, "control_options", self.control_callback, 10
        )
        self.control_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.received_messages = {}

    # Control callback should be called more frequently than timer_callback
    def control_callback(self, msg):
        self.received_messages[msg.priority] = msg.twist

        # Publish the highest priority message, i.e. the lowest priority value
        self.control_pub.publish(
            self.received_messages[min(self.received_messages.keys())]
        )

    def timer_callback(self):
        self.received_messages = {}


def main():
    rclpy.init()
    control_mux = ControlMux()
    rclpy.spin(control_mux)
    control_mux.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
