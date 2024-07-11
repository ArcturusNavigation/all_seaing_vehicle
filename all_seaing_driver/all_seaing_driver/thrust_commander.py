#!/usr/bin/env python3

from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Int64

import rclpy


class ThrustCommander(Node):

    def __init__(self):
        super().__init__("thrust_commander")

        self.declare_parameter("front_right_port", 2)
        self.declare_parameter("front_left_port", 3)
        self.declare_parameter("back_right_port", 4)
        self.declare_parameter("back_left_port", 5)

        self.front_right_port = self.get_parameter("front_right_port").value
        self.front_left_port = self.get_parameter("front_left_port").value
        self.back_right_port = self.get_parameter("back_right_port").value
        self.back_left_port = self.get_parameter("back_left_port").value

        self.create_subscription(
            Int64, "thrusters/front_right/thrust", self.front_right_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/front_left/thrust", self.front_left_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/back_right/thrust", self.back_right_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/back_left/thrust", self.back_left_cb, 10
        )
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def front_right_cb(self, msg: Int64):
        self.send_pwm(self.front_right_port, msg.data)

    def front_left_cb(self, msg: Int64):
        self.send_pwm(self.front_left_port, msg.data)

    def back_right_cb(self, msg: Int64):
        self.send_pwm(self.back_right_port, msg.data)

    def back_left_cb(self, msg: Int64):
        self.send_pwm(self.back_left_port, msg.data)

    def send_pwm(self, channel, value):
        self.get_logger().info(f"Sending PWM value {value} to channel {channel}")
        return self.proxy.call_async(
            CommandLong.Request(command=183, param1=float(channel), param2=float(value))
        )


def main(args=None):
    rclpy.init(args=args)
    node = ThrustCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
