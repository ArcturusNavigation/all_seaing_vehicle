#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class TurnOffMotors(Node):

    def __init__(self):
        super().__init__('turn_off_motors')
        print("starting node")
        self.publisher_ = self.create_publisher(Int64, '/frontleft_pwm', 10)

        self.msg = Int64()
        self.msg.data = 1500

        self.publisher_.publish(self.msg)


def main(args=None):
    print("starting turn off motors")
    rclpy.init(args=args)
    pub = TurnOffMotors()
    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
