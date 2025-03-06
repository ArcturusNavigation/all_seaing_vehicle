#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from all_seaing_interfaces.msg import ControlOption, Heartbeat
from all_seaing_interfaces.srv import GetEstopStatus

TIMER_PERIOD = 1 / 30

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

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.estop_cli = self.create_client(GetEstopStatus, "get_estop_status")
        while not self.estop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GetEstopStatus service not available, waiting again...")
            time.sleep(TIMER_PERIOD)

        self.result = None

    def timer_callback(self):
        self.future = self.estop_cli.call_async(GetEstopStatus.Request())
        self.future.add_done_callback(self.process_response)

    def process_response(self, future):
        result = future.result()

        new_mode = result.mode
        if new_mode != self.heartbeat_message.in_teleop:
            self.get_logger().info(f"Toggled teleop (now {self.heartbeat_message.in_teleop})")

        self.heartbeat_message.in_teleop = bool(new_mode)
        self.heartbeat_message.e_stopped = result.is_estopped

        if self.heartbeat_message.in_teleop and not self.heartbeat_message.e_stopped:
            self.heartbeat_publisher.publish(self.heartbeat_message)

        if not self.heartbeat_message.e_stopped:
            self.send_controls(result)

    def send_controls(self, result):
        control_option = ControlOption()
        control_option.priority = 0  # TeleOp has the highest priority value
        control_option.twist.linear.x = result.drive_y * self.joy_x_scale
        control_option.twist.linear.y = 0.0
        control_option.twist.angular.z = result.drive_x * self.joy_ang_scale
        self.control_option_pub.publish(control_option)


def main(args=None):
    rclpy.init(args=args)
    node = RoverCustomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
