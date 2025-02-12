#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node # imports Node class from ros2 packages
from all_seaing_driver.driver_library import Mechanisms, Buck
import serial
import time

# from all_seaing_interfaces.msg import ControlOption, Heartbeat
# from geometry_msgs.msg import Twist

#TODO: Write action client for perception OR figure out who to report result back to
#TODO: How to localize delivery boats and flag as delivered

def WaterDelivery(Node):
    def __init__(self):
        super().__init__("water_delivery_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")

        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.buck = Buck(self.ser)
        self.mechanisms = Mechanisms(self.ser)

        self.water_delivery_server = ActionServer(
            self,
            WaterDelivery,
            'water_delivery',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.buck.adj1_en(1)
        time.sleep(10)
        self.buck.adj1_en(0)
        result = WaterDelivery.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WaterDelivery()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
