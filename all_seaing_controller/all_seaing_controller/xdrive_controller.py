#!/usr/bin/env python3

import scipy.optimize
import rclpy
import rclpy.time
import math
import numpy as np
import scipy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class XDriveController(Node):

    def __init__(self):
        super().__init__("xdrive_controller")

        #--------------- CONSTANT PARAMETERS ---------------#

        # Thruster locations
        front_right_xy = np.array(self.declare_parameter(
            "front_right_xy", [1.1, -1.0]).get_parameter_value().double_array_value)
        back_left_xy = np.array(self.declare_parameter(
            "back_left_xy", [-2.4, 1.0]).get_parameter_value().double_array_value)
        front_left_xy = np.array(self.declare_parameter(
            "front_left_xy", [1.1, 1.0]).get_parameter_value().double_array_value)
        back_right_xy = np.array(self.declare_parameter(
            "back_right_xy", [-2.4, -1.0]).get_parameter_value().double_array_value)
        thruster_locations = np.array([front_right_xy, back_left_xy, front_left_xy, back_right_xy])

        # Distance from the center to each thruster
        thruster_distances = np.linalg.norm(thruster_locations, axis=1)

        # Angle from the center to each thruster
        thruster_loc_angles = np.arctan2(
            np.abs(thruster_locations[:,0]),
            np.abs(thruster_locations[:,1])
        )

        # Angle of the thruster in degrees (e.g. 60 if facing 15 degrees "more forward")
        thruster_angle = math.radians(self.declare_parameter(
            "thruster_angle", 45.0).get_parameter_value().double_value)

        # 0.5 * fluid density * drag coefficient * reference area (empirically determined)
        drag_x_const = self.declare_parameter(
            "drag_x_const", 5.0).get_parameter_value().double_value
        drag_y_const = self.declare_parameter(
            "drag_y_const", 30.0).get_parameter_value().double_value
        drag_z_const = self.declare_parameter(
            "drag_z_const", 20.0).get_parameter_value().double_value

        # Minimum control output
        self.min_output = self.declare_parameter(
            "min_output", -2000.0).get_parameter_value().double_value

        # Maximum control output
        self.max_output = self.declare_parameter(
            "max_output", 2000.0).get_parameter_value().double_value
        
        # From the T200 datasheet, approximately 40N maximum force
        THRUST_CONST = 40.0
        thrust_force_x = THRUST_CONST * math.sin(thruster_angle)
        thrust_force_y = THRUST_CONST * math.cos(thruster_angle)
        thrust_torque = THRUST_CONST * thruster_distances * np.sin(thruster_angle + thruster_loc_angles)

        # Matrix of thrust forces and torques
        self.thrust_forces = np.array([
            [thrust_force_x, thrust_force_x, thrust_force_x, thrust_force_x],
            [thrust_force_y, thrust_force_y, -thrust_force_y, -thrust_force_y],
            [thrust_torque[0], -thrust_torque[1], -thrust_torque[2], thrust_torque[3]],
        ])

        # Matrix of drag constants
        self.drag_constants = np.array([
            [drag_x_const, 0, 0],
            [0, drag_y_const, 0],
            [0, 0, drag_z_const],
        ])

        #--------------- SUBSCRIBERS, PUBLISHERS, AND TIMERS ---------------#

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.front_right_pub = self.create_publisher(Float64, "thrusters/front_right/thrust", 10)
        self.back_left_pub = self.create_publisher(Float64, "thrusters/back_left/thrust", 10)
        self.front_left_pub = self.create_publisher(Float64, "thrusters/front_left/thrust", 10)
        self.back_right_pub = self.create_publisher(Float64, "thrusters/back_right/thrust", 10)
    
    def calculate_control_output(self, target_vel):
        target_vel_sq = np.sign(target_vel) * np.square(target_vel)
        constraint_bounds = self.drag_constants @ target_vel_sq
        result = scipy.optimize.minimize(
            lambda u: np.linalg.norm(u),
            np.ones(4),
            constraints=scipy.optimize.LinearConstraint(
                self.thrust_forces, lb=constraint_bounds, ub=constraint_bounds
            )
        )

        if not result.success:
            self.get_logger().error("Optimization failed to converge!")
            return None
        
        control_output = result.x.reshape(4)
        if np.any(np.abs(control_output) > 1):
            control_output = control_output / np.max(np.abs(control_output))
        return control_output

    def cmd_vel_cb(self, msg: Twist):
        target_vel = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        ])
        control_output = self.calculate_control_output(target_vel)

        # Don't respond if optimization failed to converge
        if control_output is None:
            return
        
        # Scale control_output from [-1,1] to [min_output,max_output]
        output_range_mean = (self.max_output + self.min_output) / 2
        scaled_control_output = control_output * (self.max_output - self.min_output) / 2
        thrust_cmd = scaled_control_output + output_range_mean

        # Publish thrust commands
        self.front_right_pub.publish(Float64(data=thrust_cmd[0]))
        self.back_left_pub.publish(Float64(data=thrust_cmd[1]))
        self.front_left_pub.publish(Float64(data=thrust_cmd[2]))
        self.back_right_pub.publish(Float64(data=thrust_cmd[3]))


def main(args=None):
    rclpy.init(args=args)
    node = XDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
