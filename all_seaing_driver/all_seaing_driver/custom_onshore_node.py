#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_driver.central_hub import ESTOP
import serial

from all_seaing_interfaces.msg import ControlOption, Heartbeat

HEART_RATE = 1

class CustomOnshoreNode(Node):
    def __init__(self):
        super().__init__("manual_controller")

        self.declare_parameter("joy_x_scale", 2.0)
        self.declare_parameter("joy_y_scale", -1.0)
        self.declare_parameter("joy_ang_scale", -0.8)
        self.declare_parameter("serial_port", "/dev/ttyACM0")

        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.estop = ESTOP(self.ser)

        self.control_option_pub = self.create_publisher(
            ControlOption, "control_options", 10
        )

        self.heartbeat_message = Heartbeat()
        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeat", 10)
        self.heartbeat_message.in_teleop = True
        self.heartbeat_message.e_stopped = False

        self.heartbeat_timer = self.create_timer(HEART_RATE, self.timer_callback)

        self.get_logger().info("Starting onshore node, teleop enabled")

    def timer_callback(self):
        self.heartbeat_message.in_teleop = bool(self.estop.mode())
        self.heartbeat_message.e_stopped = bool(self.estop.estop())
        self.heartbeat_publisher.publish(self.heartbeat_message)

        if self.heartbeat_message.e_stopped:
            self.get_logger().fatal("E-STOP ACTIVATED :<")
            return

        self.send_controls()

    def send_controls(self):
        control_option = ControlOption()
        control_option.priority = 0  # TeleOp has the highest priority value
        control_option.twist.linear.x = self.estop.drive_y()
        control_option.twist.linear.y = 0.0
        control_option.twist.angular.z = self.estop.drive_x()
        self.control_option_pub.publish(control_option)


def main(args=None):
    rclpy.init(args=args)
    node = CustomOnshoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
