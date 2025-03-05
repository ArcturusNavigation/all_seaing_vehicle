#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_driver.central_hub import ESTOP
import serial

from all_seaing_interfaces.msg import ControlOption, Heartbeat

HEART_RATE = 1
CONTROL_RATE = 1 / 30

class RoverCustomController(Node):
    def __init__(self):
        super().__init__("rover_custom_controller")

        self.joy_x_scale = self.declare_parameter(
            "joy_x_scale", 2.0).get_parameter_value().double_value
        self.joy_ang_scale = self.declare_parameter(
            "joy_ang_scale", 0.8).get_parameter_value().double_value
        self.declare_parameter("serial_port", "/dev/ttyACM0")

        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.estop = ESTOP(self.ser)

        self.control_option_pub = self.create_publisher(
            ControlOption, "control_options", 10
        )

        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeat", 10)
        self.heartbeat_message = Heartbeat()
        self.heartbeat_message.in_teleop = True
        self.heartbeat_message.e_stopped = False

        self.heartbeat_timer = self.create_timer(HEART_RATE, self.heart_timer_callback)
        self.controls_timer = self.create_timer(CONTROL_RATE, self.controls_timer_callback)

    def heart_timer_callback(self):
        new_mode = bool(self.estop.mode())
        if new_mode != self.heartbeat_message.in_teleop:
            self.get_logger().info(f"Toggled teleop (now {self.heartbeat_message.in_teleop})")

        self.heartbeat_message.in_teleop = new_mode
        self.heartbeat_message.e_stopped = bool(self.estop.estop())

        if self.heartbeat_message.in_teleop and not self.heartbeat_message.e_stopped:
            self.heartbeat_publisher.publish(self.heartbeat_message)

    def controls_timer_callback(self):
        if not self.heartbeat_message.e_stopped:
            self.send_controls()

    def send_controls(self):
        control_option = ControlOption()
        control_option.priority = 0  # TeleOp has the highest priority value
        control_option.twist.linear.x = self.estop.drive_y() * self.joy_x_scale
        control_option.twist.linear.y = 0.0
        control_option.twist.angular.z = self.estop.drive_x() * self.joy_ang_scale
        self.control_option_pub.publish(control_option)


def main(args=None):
    rclpy.init(args=args)
    node = RoverCustomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
