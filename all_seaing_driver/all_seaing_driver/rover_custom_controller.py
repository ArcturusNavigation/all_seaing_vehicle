#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from all_seaing_interfaces.msg import ControlOption, Heartbeat
from all_seaing_interfaces.srv import GetEstopStatus

HEART_RATE = 1
CONTROL_RATE = 1 / 30

class RoverCustomController(Node):
    def __init__(self):
        super().__init__("rover_custom_controller")

        self.joy_x_scale = self.declare_parameter(
            "joy_x_scale", 2.0).get_parameter_value().double_value
        self.joy_ang_scale = self.declare_parameter(
            "joy_ang_scale", 0.8).get_parameter_value().double_value

        self.control_option_pub = self.create_publisher(
            ControlOption, "control_options", 10
        )

        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeat", 10)
        self.heartbeat_message = Heartbeat()
        self.heartbeat_message.in_teleop = True
        self.heartbeat_message.e_stopped = False

        self.heartbeat_timer = self.create_timer(HEART_RATE, self.heart_timer_callback)
        self.controls_timer = self.create_timer(CONTROL_RATE, self.controls_timer_callback)

        self.estop_cli = self.create_client(GetEstopStatus, "get_estop_status")
        while not self.estop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetEstopStatus service not available, waiting again...")
            time.sleep(CONTROL_RATE)

        self.result = None

    def heart_timer_callback(self):
        self.future = self.estop_cli.call_async(GetEstopStatus.Request())
        rclpy.spin_until_future_complete(self, self.future)
        self.result = self.future.result()

        new_mode = bool(self.result.mode)
        if new_mode != self.heartbeat_message.in_teleop:
            self.get_logger().info(f"Toggled teleop (now {self.heartbeat_message.in_teleop})")

        self.heartbeat_message.in_teleop = new_mode
        self.heartbeat_message.e_stopped = bool(self.result.is_estopped)

        if self.heartbeat_message.in_teleop and not self.heartbeat_message.e_stopped:
            self.heartbeat_publisher.publish(self.heartbeat_message)

    def controls_timer_callback(self):
        if not self.heartbeat_message.e_stopped:
            self.send_controls()

    def send_controls(self):
        if self.result is None:
            return

        control_option = ControlOption()
        control_option.priority = 0  # TeleOp has the highest priority value
        control_option.twist.linear.x = self.result.drive_y * self.joy_x_scale
        control_option.twist.linear.y = 0.0
        control_option.twist.angular.z = self.result.drive_x * self.joy_ang_scale
        self.control_option_pub.publish(control_option)


def main(args=None):
    rclpy.init(args=args)
    node = RoverCustomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
