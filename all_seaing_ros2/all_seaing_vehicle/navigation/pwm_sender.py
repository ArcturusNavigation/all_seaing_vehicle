#!/usr/bin/env python3
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Int64

import rclpy
from rclpy.node import Node

class pwm_sender(Node):

    def __init__(self):
        super().__init__("pwm_subscriber")
        self.pose_subscriber_left = self.create_subscription(Int64, "frontleft_pwm" , self.pwm_callback_FL, 10)
        self.pose_subscriber_right = self.create_subscription(Int64, "frontright_pwm" ,self.pwm_callback_FR, 10)
        self.pose_subscriber_right = self.create_subscription(Int64, "backleft_pwm" ,self.pwm_callback_BL, 10)
        self.pose_subscriber_right = self.create_subscription(Int64, "backright_pwm" ,self.pwm_callback_BR, 10)
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def pwm_callback_FR(self, msg: Int64):
        self.message = self.get_logger().info(str(msg))
        self.send_pwm(2, msg.data)

    def pwm_callback_FL(self,msg: Int64):
        self.message = self.get_logger().info(str(msg))
        self.send_pwm(3, msg.data)

    def pwm_callback_BR(self, msg: Int64):
        self.message = self.get_logger().info(str(msg))
        self.send_pwm(4, msg.data)

    def pwm_callback_BL(self, msg: Int64):
        self.message = self.get_logger().info(str(msg))
        self.send_pwm(5, msg.data)

    def send_pwm(self, channel, value):
        print(f"Sending pwm value {value} to channel {channel}")
        return self.proxy.call_async(CommandLong.Request(
            command=183,
            param1=float(channel),
            param2=float(value)
        ))

def main(args=None):
    rclpy.init(args=args)
    node = pwm_sender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
