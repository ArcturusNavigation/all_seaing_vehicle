#!/usr/bin/env python3

from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Int64

import rclpy


class ThrustCommander(Node):

    def __init__(self):
        super().__init__("thrust_commander")

        self.declare_parameter("frontright_port", 2)
        self.declare_parameter("frontleft_port", 3)
        self.declare_parameter("backright_port", 4)
        self.declare_parameter("backleft_port", 5)

        self.frontright_port = self.get_parameter("frontright_port").value
        self.frontleft_port = self.get_parameter("frontleft_port").value
        self.backright_port = self.get_parameter("backright_port").value
        self.backleft_port = self.get_parameter("backleft_port").value

        self.create_subscription(Int64, "frontright_pwm", self.pwm_callback_FR, 10)
        self.create_subscription(Int64, "frontleft_pwm", self.pwm_callback_FL, 10)
        self.create_subscription(Int64, "backright_pwm", self.pwm_callback_BR, 10)
        self.create_subscription(Int64, "backleft_pwm", self.pwm_callback_BL, 10)
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def pwm_callback_FR(self, msg: Int64):
        self.send_pwm(self.frontright_port, msg.data)

    def pwm_callback_FL(self, msg: Int64):
        self.send_pwm(self.frontleft_port, msg.data)

    def pwm_callback_BR(self, msg: Int64):
        self.send_pwm(self.backright_port, msg.data)

    def pwm_callback_BL(self, msg: Int64):
        self.send_pwm(self.backleft_port, msg.data)

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
