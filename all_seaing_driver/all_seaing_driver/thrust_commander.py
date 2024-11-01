#!/usr/bin/env python3

from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Float64

import rclpy
from all_seaing_interfaces.msg import Heartbeat

class ThrustCommander(Node):

    def __init__(self):
        super().__init__("thrust_commander")

        self.front_right_port = self.declare_parameter(
            "front_right_port", 2).get_parameter_value().integer_value
        self.front_left_port = self.declare_parameter(
            "front_left_port", 3).get_parameter_value().integer_value
        self.back_right_port = self.declare_parameter(
            "back_right_port", 4).get_parameter_value().integer_value
        self.back_left_port = self.declare_parameter(
            "back_left_port", 5).get_parameter_value().integer_value
        self.heartbeat_threshold = self.declare_parameter(
            "required_heartbeat_recentness", 3.0).get_parameter_value().double_value

        self.prev_heartbeat = self.get_clock().now()
        self.e_stopped = False

        self.front_right_sub = self.create_subscription(
            Float64, "thrusters/front_right/thrust", self.front_right_cb, 10)
        self.front_left_sub = self.create_subscription(
            Float64, "thrusters/front_left/thrust", self.front_left_cb, 10)
        self.back_right_sub = self.create_subscription(
            Float64, "thrusters/back_right/thrust", self.back_right_cb, 10)
        self.back_left_sub = self.create_subscription(
            Float64, "thrusters/back_left/thrust", self.back_left_cb, 10)

        self.front_right_command = 1500
        self.front_left_command = 1500
        self.back_right_command = 1500
        self.back_left_command = 1500

        self.heartbeat_sub = self.create_subscription(Heartbeat, "heartbeat", self.receive_heartbeat, 10)
        self.timer = self.create_timer(1 / 8, self.timer_callback)
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

    def front_right_cb(self, msg: Float64):
        self.front_right_command = msg.data

    def front_left_cb(self, msg: Float64):
        self.front_left_command = msg.data

    def back_right_cb(self, msg: Float64):
        self.back_right_command = msg.data

    def back_left_cb(self, msg: Float64):
        self.back_left_command = msg.data

    def turn_off_thrusters(self):
        self.front_right_command = 1500
        self.front_left_command = 1500
        self.back_right_command = 1500
        self.back_left_command = 1500
        
    def send_pwm(self, channel: int, value: float):
        self.get_logger().debug(f"Sending PWM value {value} to channel {channel}")
        return self.proxy.call_async(
            CommandLong.Request(command=183, param1=float(channel), param2=float(value))
        )
    
    def receive_heartbeat(self, msg):
        if self.e_stopped:
            self.get_logger().info("Regained heartbeat!")

        self.get_logger().debug("Heartbeat received!")
        self.prev_heartbeat = self.get_clock().now()
        self.e_stopped = msg.e_stopped

    def timer_callback(self):
        time_since_last_heartbeat = (self.get_clock().now() - self.prev_heartbeat).nanoseconds / 1e9

        if time_since_last_heartbeat > self.heartbeat_threshold and not self.e_stopped:
            self.e_stopped = True

        if self.e_stopped:
            self.get_logger().warning("Lost heartbeat! Entering e-stop mode.")
            self.turn_off_thrusters()
        
        self.send_pwm(self.front_right_port, self.front_right_command)
        self.send_pwm(self.front_left_port, self.front_left_command)
        self.send_pwm(self.back_right_port, self.back_right_command)
        self.send_pwm(self.back_left_port, self.back_left_command)


def main(args=None):
    rclpy.init(args=args)
    node = ThrustCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
