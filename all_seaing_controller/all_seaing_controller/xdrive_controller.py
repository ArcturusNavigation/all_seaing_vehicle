#!/usr/bin/env python3
import math
import numpy as np
import rclpy
import rclpy.time
import scipy.optimize
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

TIMER_PERIOD = 1 / 60

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
        self.drag_constants = np.diag(np.array(self.declare_parameter(
            "drag_constants", [5.0, 5.0, 40.0]).get_parameter_value().double_array_value))

        # Control output range
        self.output_range = np.array(self.declare_parameter(
            "output_range", [-1500.0, 1500.0]).get_parameter_value().double_array_value)

        # Smoothing factor with 0 corresponding to no smoothing
        self.smoothing = self.declare_parameter(
            "smoothing_factor", 0.8).get_parameter_value().double_value
        self.curr_output = np.zeros(4)
        self.prev_output = np.zeros(4)
        
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

        #--------------- SUBSCRIBERS, PUBLISHERS, AND TIMERS ---------------#

        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.front_right_pub = self.create_publisher(Float64, "thrusters/front_right/thrust", 10)
        self.back_left_pub = self.create_publisher(Float64, "thrusters/back_left/thrust", 10)
        self.front_left_pub = self.create_publisher(Float64, "thrusters/front_left/thrust", 10)
        self.back_right_pub = self.create_publisher(Float64, "thrusters/back_right/thrust", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_cb)
    
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

        self.curr_output = control_output

    def timer_cb(self):
        """
        Using a separate timer callback allows consistent smoothing even if the callback gets called
        at different rates depending on how fast cmd_vel gets published by other nodes.
        """

        # Smoothing via low-pass filter
        control_output = self.smoothing * self.prev_output + (1 - self.smoothing) * self.curr_output
        self.prev_output = control_output
        
        # Scale control_output from [-1,1] to output range
        scaled_control_output = control_output * (self.output_range[1] - self.output_range[0]) / 2
        thrust_cmd = scaled_control_output + np.mean(self.output_range)

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
