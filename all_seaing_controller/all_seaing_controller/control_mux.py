#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlOption
from geometry_msgs.msg import Twist

ZERO_CMD_TIMER_PERIOD = 1 / 4


class ControlMux(Node):
    def __init__(self):
        super().__init__("control_mux")

        self.control_sub = self.create_subscription(
            ControlOption, "control_options", self.control_callback, 10
        )
        self.control_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(ZERO_CMD_TIMER_PERIOD, self.timer_callback)
        self.received_priorities = set()
        self.received_messages = {}

    # Control callback should be called more frequently than timer_callback
    def control_callback(self, msg):
        self.received_priorities.add(msg.priority)
        self.received_messages[msg.priority] = msg.twist

        # Publish the highest priority message, i.e. the lowest priority value
        self.control_pub.publish(
            self.received_messages[min(self.received_messages.keys())]
        )

    def timer_callback(self):
        # "Do-nothing" command sent at 4Hz for safety
        self.control_pub.publish(Twist())

        # Remove from received messages if not received during timer period
        for p in self.received_messages.keys() - self.received_priorities:
            del self.received_messages[p]
        self.received_priorities = set()


def main():
    rclpy.init()
    control_mux = ControlMux()
    rclpy.spin(control_mux)
    control_mux.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
