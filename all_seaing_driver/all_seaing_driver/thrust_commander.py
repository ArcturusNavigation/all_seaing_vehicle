#!/usr/bin/env python3

from rclpy.node import Node
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Int64

import rclpy
from all_seaing_interfaces.msg import Heartbeat
from all_seaing_interfaces.msg import ControlMessage

class ThrustCommander(Node):

    def __init__(self):
        super().__init__("thrust_commander")

        self.declare_parameter("front_right_port", 2)
        self.declare_parameter("front_left_port", 3)
        self.declare_parameter("back_right_port", 4)
        self.declare_parameter("back_left_port", 5)

        self.front_right_port = self.get_parameter("front_right_port").value
        self.front_left_port = self.get_parameter("front_left_port").value
        self.back_right_port = self.get_parameter("back_right_port").value
        self.back_left_port = self.get_parameter("back_left_port").value

        self.last_heartbeat_timestamp = self.get_time()
        self.required_heartbeat_recentness = 3
        self.is_estopped = False

        self.create_subscription(
            Int64, "thrusters/front_right/thrust", self.front_right_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/front_left/thrust", self.front_left_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/back_right/thrust", self.back_right_cb, 10
        )
        self.create_subscription(
            Int64, "thrusters/back_left/thrust", self.back_left_cb, 10
        )
        self.proxy = self.create_client(CommandLong, "/mavros/cmd/command")

        self.create_subscription(Heartbeat, "heartbeat", self.receive_heartbeat, 10)

        timer_period = (
            1 / 8
        )  # update rate for the output loop
        self.timer = self.create_timer(
            timer_period, self.timer_callback
        )  # start the output loop

    def front_right_cb(self, msg: Int64):
        self.send_pwm(self.front_right_port, msg.data)

    def front_left_cb(self, msg: Int64):
        self.send_pwm(self.front_left_port, msg.data)

    def back_right_cb(self, msg: Int64):
        self.send_pwm(self.back_right_port, msg.data)

    def back_left_cb(self, msg: Int64):
        self.send_pwm(self.back_left_port, msg.data)
    
    def get_time(self):
        """
        Get the current time. Written into a function because it's annoying to write every time.
        """
        return self.get_clock().now()
    
    def turn_off_thrusters(self):
        """
        Turn off all thrusters.
        """
        self.send_pwm(self.front_right_port, 1500)
        self.send_pwm(self.front_left_port, 1500)
        self.send_pwm(self.back_right_port, 1500)
        self.send_pwm(self.back_left_port, 1500)

    def send_pwm(self, channel, value):
        self.get_logger().info(f"Sending PWM value {value} to channel {channel}")
        return self.proxy.call_async(
            CommandLong.Request(command=183, param1=float(channel), param2=float(value))
        )
    
    def get_thrust_values(self, tx, ty, tn):
        """
        Given linear and angular velocities, get thrust velocities for each thruster.
        """
        d = 2 ** (3 / 2)
        ang = tn / (4 * self.r)
        return {
            "back_right": (tx - ty) / d + ang,
            "back_left": (ty + tx) / d - ang,
            "front_left": (tx - ty) / d - ang,
            "front_right": (ty + tx) / d + ang,
        }
    
    def receive_heartbeat(self, msg):
        """
        Callback function for receiving hearbeat messages, which are necessary for the controller to run.
        """
        self.get_logger().info("Heartbeat received!")
        if msg.e_stopped: 
            self.turn_off_thrusters()
            raise Exception("Received e-stop message! Killing node")

        self.last_heartbeat_timestamp = self.get_time()

    def restrict_input(self, input, max_val):
        if input < -max_val:
            return -max_val
        if input > max_val:
            return max_val
        return input
    
    def timer_callback(self):
        current_time = self.get_time()
        time_since_last_heartbeat = (current_time - self.last_heartbeat_timestamp).nanoseconds / 1e9

        if time_since_last_heartbeat > self.required_heartbeat_recentness:
            # Heartbeat timeout - stop thrusters
            self.turn_off_thrusters()
            raise Exception("Lost heartbeat! Killing node")

        self.get_logger().info(f"Heartbeat OK. Time since last heartbeat: {time_since_last_heartbeat:.2f}s")

    def update_control_input(self, msg: ControlMessage):
        """Update control input when received."""
        self.get_logger().info(f"Received control input: x={msg.x}, y={msg.y}, angular={msg.angular}")
        # Control input handling logic can be added here if necessary


def main(args=None):
    rclpy.init(args=args)
    node = ThrustCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
