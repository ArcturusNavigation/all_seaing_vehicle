#!/usr/bin/env python3
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Int64

import rclpy

from all_seaing_vehicle.utils.e_stopped_node import EStoppedNode

class pwm_sender(EStoppedNode):

    def __init__(self):
        super().__init__("pwm_subscriber")

        self.declare_parameter("frontright_port", 2)
        self.declare_parameter("frontleft_port", 3)
        self.declare_parameter("backright_port", 4)
        self.declare_parameter("backleft_port", 5)

        self.frontright_port = self.get_parameter("frontright_port").value
        self.frontleft_port = self.get_parameter("frontleft_port").value
        self.backright_port = self.get_parameter("backright_port").value
        self.backleft_port = self.get_parameter("backleft_port").value

        self.create_subscription(Int64, "frontright_pwm", self.pwm_callback_FR, 10)
        self.create_subscription(Int64, "frontleft_pwm" , self.pwm_callback_FL, 10)
        self.create_subscription(Int64, "backright_pwm",  self.pwm_callback_BR, 10)
        self.create_subscription(Int64, "backleft_pwm",   self.pwm_callback_BL, 10)
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def pwm_callback_FR(self, msg: Int64):
        self.send_pwm(self.frontright_port, msg.data)

    def pwm_callback_FL(self,msg: Int64):
        self.send_pwm(self.frontleft_port, msg.data)

    def pwm_callback_BR(self, msg: Int64):
        self.send_pwm(self.backright_port, msg.data)

    def pwm_callback_BL(self, msg: Int64):
        self.send_pwm(self.backleft_port, msg.data)

    def send_pwm(self, channel, value):
        if not self.is_e_stopped:
            self.get_logger().info(f"Sending PWM value {value} to channel {channel}")
            self.last_received_time = self.get_clock().now().to_msg().sec
            return self.proxy.call_async(CommandLong.Request(
                command=183,
                param1=float(channel),
                param2=float(value)
            ))
        self.get_logger().info("Cannot send PWM as node is E-stopped!")

def main(args=None):
    rclpy.init(args=args)
    node = pwm_sender()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
