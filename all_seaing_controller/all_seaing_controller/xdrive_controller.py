#!/usr/bin/env python3

import rclpy
import rclpy.time
import math
from tf_transformations import euler_from_quaternion
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.msg import ControlMessage


class XDriveController(Node):

    def __init__(self):
        super().__init__("xdrive_controller")

        l = self.declare_parameter(
            "boat_length", 3.5).get_parameter_value().double_value
        w = self.declare_parameter(
            "boat_width", 2.0).get_parameter_value().double_value
        min_output = self.declare_parameter(
            "min_output", -1400.0).get_parameter_value().double_value
        max_output = self.declare_parameter(
            "max_output", 1400.0).get_parameter_value().double_value
        timer_period = self.declare_parameter(
            "timer_period", 1/8).get_parameter_value().double_value

        self.thrust_factor = (max_output - min_output)
        self.midpoint = (max_output + min_output) / 2
        self.r = ((l**2 + w**2) / 2 - l*w) ** 0.5 / 2

        self.x = 0
        self.y = 0
        self.theta = 0
        self.local_vx = 0
        self.local_vy= 0
        self.global_vx = 0
        self.global_vy= 0
        self.omega = 0

        pid_x = PIDController(0.1, 0, 0)
        pid_y = PIDController(0.1, 0, 0)
        pid_theta = PIDController(1, 0, 0)
        pid_vx = PIDController(1, 0, 0)
        pid_vy = PIDController(1, 0, 0)
        pid_omega = PIDController(2, 0.5, 0)
        do_nothing_pid = PIDController(0, 0, 0)

        self.x_controllers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_x,
            ControlMessage.WORLD_VELOCITY: pid_vx,
            ControlMessage.LOCAL_VELOCITY: pid_vx,
        }

        self.y_controllers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_y,
            ControlMessage.WORLD_VELOCITY: pid_vy,
            ControlMessage.LOCAL_VELOCITY: pid_vy,
        }

        self.angular_controllers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_theta,
            ControlMessage.WORLD_VELOCITY: pid_omega,
            ControlMessage.LOCAL_VELOCITY: pid_omega,
        }

        self.curr_angular_mode = ControlMessage.OFF
        self.curr_linear_mode = ControlMessage.OFF
        self.prev_update_time = self.get_clock().now()

        self.control_sub = self.create_subscription(
            ControlMessage, "control_input", self.control_msg_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "odometry/filtered", self.odom_cb, 10)
        
        self.thrust_topic_names = (
            "thrusters/front_left/thrust",
            "thrusters/front_right/thrust",
            "thrusters/back_left/thrust",
            "thrusters/back_right/thrust",
        )

        self.thrust_publishers = {}
        for topic in self.thrust_topic_names:
            self.thrust_publishers[topic] = self.create_publisher(
                Float64, topic, 10
            )

        self.timer = self.create_timer(timer_period, self.control_loop)

    def get_thrust_values(self, target_x, target_y, target_angular):
        d = 2 ** (3 / 2)
        ang = target_angular / (4 * self.r)
        return {
            self.back_right_name: (target_x - target_y) / d + ang,
            self.back_left_name: (target_y + target_x) / d - ang,
            self.front_left_name: (target_x - target_y) / d - ang,
            self.front_right_name: (target_y + target_x) / d + ang,
        }

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.local_vx = msg.twist.twist.linear.x
        self.local_vy = msg.twist.twist.linear.y
        self.omega = msg.twist.twist.angular.z
        _, _, self.theta = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])
        self.global_vx = self.local_vx * math.cos(self.theta) - self.local_vy * math.sin(self.theta)
        self.global_vy = self.local_vy * math.cos(self.theta) + self.local_vx * math.sin(self.theta)

    def convert_to_pwm_and_send(self, thrust_values):
        for topic in self.thrust_topic_names:
            float_msg = Float64()
            float_msg.data = thrust_values[topic] * self.thrust_factor + self.midpoint
            self.thrust_publishers[topic].publish(float_msg)

    def update_controllers(self):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.x_controllers[ControlMessage.WORLD_POSITION].update(self.x, dt)
        self.x_controllers[ControlMessage.WORLD_VELOCITY].update(self.global_vx, dt)
        self.x_controllers[ControlMessage.LOCAL_VELOCITY].update(self.local_vx, dt)
        self.y_controllers[ControlMessage.WORLD_POSITION].update(self.y, dt)
        self.y_controllers[ControlMessage.WORLD_VELOCITY].update(self.global_vy, dt)
        self.y_controllers[ControlMessage.LOCAL_VELOCITY].update(self.local_vy, dt)
        self.angular_controllers[ControlMessage.WORLD_POSITION].update(self.theta, dt)
        self.angular_controllers[ControlMessage.WORLD_VELOCITY].update(self.omega, dt)
        self.angular_controllers[ControlMessage.LOCAL_VELOCITY].update(self.omega, dt)

    def control_loop(self):
        self.update_controllers()
        x_output = self.x_controllers[self.curr_linear_mode].get_effort()
        y_output = self.y_controllers[self.curr_linear_mode].get_effort()
        angular_output = self.angular_controllers[self.curr_angular_mode].get_effort()

        if (self.curr_linear_mode == ControlMessage.WORLD_VELOCITY
            or self.curr_linear_mode == ControlMessage.WORLD_POSITION):
            x_boat_space = x_output * math.cos(self.theta) + y_output * math.sin(self.theta)
            y_boat_space = y_output * math.cos(self.theta) - x_output * math.sin(self.theta)
        else:
            x_boat_space = x_output
            y_boat_space = y_output

        results = self.get_thrust_values(x_boat_space, y_boat_space, angular_output)
        self.convert_to_pwm_and_send(results)

        self.prev_update_time = self.get_clock().now()

    def control_msg_cb(self, msg: ControlMessage):
        if msg.linear_control_mode != self.curr_linear_mode:
            self.x_controllers[msg.linear_control_mode].reset()
            self.y_controllers[msg.linear_control_mode].reset()

        if msg.angular_control_mode != self.curr_angular_mode:
            self.angular_controllers[msg.angular_control_mode].reset()

        for controller in self.x_controllers.values():
            controller.set_setpoint(msg.x)
        for controller in self.y_controllers.values():
            controller.set_setpoint(msg.y)
        for controller in self.angular_controllers.values():
            controller.set_setpoint(msg.angular)

        self.curr_linear_mode = msg.linear_control_mode
        self.curr_angular_mode = msg.angular_control_mode


def main(args=None):
    rclpy.init(args=args)
    xdrive_control_node = XDriveController()
    rclpy.spin(xdrive_control_node)
    xdrive_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
