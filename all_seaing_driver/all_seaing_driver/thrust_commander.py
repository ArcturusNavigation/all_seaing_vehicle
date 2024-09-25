#!/usr/bin/env python3

from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Float64

import rclpy


class ThrustCommander(Node):

    def __init__(self):
        super().__init__("thrust_commander")

        self.front_right_port = self.declare_parameter(
            "front_right_port", 2).get_parameter_value().integer_value
        self.front_left_port = self.declare_parameter(
            "front_left_port", 3).get_parameter_value().integer_value
        self.back_right_port = self.declare_parameter(
            "back_right_port", 4).get_parameter_value().integer_value
        self.back_left_port = self.declare_parameter(
            "back_left_port", 5).get_parameter_value().integer_value

        self.front_right_sub = self.create_subscription(
            Float64, "thrusters/front_right/thrust", self.front_right_cb, 10)
        self.front_left_sub = self.create_subscription(
            Float64, "thrusters/front_left/thrust", self.front_left_cb, 10)
        self.back_right_sub = self.create_subscription(
            Float64, "thrusters/back_right/thrust", self.back_right_cb, 10)
        self.back_left_sub = self.create_subscription(
            Float64, "thrusters/back_left/thrust", self.back_left_cb, 10)
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def front_right_cb(self, msg: Float64):
        self.send_pwm(self.front_right_port, msg.data)

    def front_left_cb(self, msg: Float64):
        self.send_pwm(self.front_left_port, msg.data)

    def back_right_cb(self, msg: Float64):
        self.send_pwm(self.back_right_port, msg.data)

    def back_left_cb(self, msg: Float64):
        self.send_pwm(self.back_left_port, msg.data)

    def send_pwm(self, channel: int, value: float):
        self.get_logger().debug(f"Sending PWM value {value} to channel {channel}")
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
