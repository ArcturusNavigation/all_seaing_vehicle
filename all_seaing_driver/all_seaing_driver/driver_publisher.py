#!/usr/bin/env python3

import rclpy
from rclpy.node import Node # imports Node class from ros2 packages
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from all_seaing_driver.driver_lib import ESTOP
import serial
import time

from all_seaing_interfaces.msg import ControlOption
from geometry_msgs.msg import Twist

TIMER_RATE = 1

class DriverPublisher(Node):
    def __init__(self):
        super().__init__("driver_publisher")

        self.declare_parameter("joy_x_scale", 2.0)
        self.declare_parameter("joy_y_scale", -1.0)
        self.declare_parameter("joy_ang_scale", -0.8)
        self.declare_parameter("serial_port", "COM16")

        self.joy_x_scale = self.get_parameter("joy_x_scale").value
        self.joy_y_scale = self.get_parameter("joy_y_scale").value
        self.joy_ang_scale = self.get_parameter("joy_ang_scale").value
        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.estop = ESTOP(self.ser)

        self.estop_msg = UInt8()
        self.estop_pub = self.create_publisher(UInt8, "estop_stat", 10)
        self.estop_msg.e_stopped = 0

        self.enter_held = False

        self.control_option_pub = self.create_publisher(
            ControlOption, "control_options", 10
        )
        self.joy_control_sub = self.create_subscription(
            Joy, "/joy", self.joystick_callback, 10
        )
        self.estop_stat_timer = self.create_timer(TIMER_RATE, self.send_estop_stat)

        self.get_logger().info("Starting driver node, teleop enabled")

    def send_estop_stat(self):
        self.estop_pub.publish(self.estop_msg)

    def send_controls(self, x, y, angular):
        control_option = ControlOption()
        control_option.priority = 0  # TeleOp has the highest priority value
        control_option.twist.linear.x = self.estop.drive_x()[0]
        control_option.twist.linear.y = self.estop.drive_y()[0]
        control_option.twist.angular.z = angular
        self.control_option_pub.publish(control_option)

    def joystick_callback(self, msg):
        self.estop_msg.e_stopped = self.estop.estop()

        if self.estop_msg.e_stopped == 1:
            self.get_logger().info("E-stop activated.")
            self.estop_pub.publish(self.estop_msg)
            return
        else:
            self.send_controls(
                msg.axes[1] * self.joy_x_scale,
                msg.axes[0] * self.joy_y_scale,
                msg.axes[2] * self.joy_ang_scale,
            )


def main(args=None):
    rclpy.init(args=args)
    node = DriverPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
