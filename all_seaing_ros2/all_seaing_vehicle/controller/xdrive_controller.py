#!/usr/bin/env python3
import rclpy
import time
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry

from all_seaing_interfaces.msg import ControlMessage

class PID:
    """
    A data class representing a PID object.
    """
    def __init__(self, p, i, d, default_value = 0):
        self.p = p
        self.i = i
        self.d = d
        self.feedback = None
        self.default_value = default_value
        self.accumulated_error = 0
        self.previousError = None
    def update_feedback_value(self, value):
        self.feedback = value
    def determine_output(self, input, dt):
        if self.feedback is None:
            return self.default_value
        error = input - self.feedback
        print("error: ", error)
        self.accumulated_error += error * dt
        derror_dt = 0 if self.previousError is None else (error - self.previousError)/dt
        self.previousError = error
        return self.p * error + self.i * self.accumulated_error + self.d * derror_dt
    def reset(self):
        self.accumulated_error = 0
        self.previousError = None

class PIDSwitcher:
    def __init__(self, true_pid, false_pid, default_value = 0):
        self.true_pid = true_pid
        self.false_pid = false_pid
        self.mode = True
        self.pid_input = None
        self.default_value = default_value
    def update_mode(self, mode):
        if mode != self.mode:
            self.true_pid.reset()
            self.false_pid.reset()
        self.mode = mode
    def update_input(self, pid_input):
        self.pid_input = pid_input
    def determine_output(self, dt):
        if self.pid_input is None:
            return self.default_value
        pid = self.true_pid if self.mode else self.false_pid
        return pid.determine_output(self.pid_input, dt)


class Controller(Node):
    """
    A simple controller for x-drive. Receives velocities and/or heading as input
    and gives PWM output to thrusters.
    """
    def __init__(self):
        """
        Initialize the controller.
        """
        super().__init__("xdrive_controller") # initialize ROS2

        self.declare_parameter("in_sim", False)
        in_sim = bool(self.get_parameter("in_sim").value)
        print("in sim: ", in_sim)
        
        self.declare_parameter("debug", False)
        self.debug_mode = bool(self.get_parameter("debug").value)

        l = 3.5 # BOAT LENGTH
        w = 2 # BOAT WIDTH
        self.msg_type = Float64 if in_sim else Int64
        self.py_type = float if in_sim else int
        min_output = -1000 if in_sim else 1100 # the minimum PWM output value for thrusters
        max_output = 1000 if in_sim else 1900 # the maximum PWM output value for thrusters
        self.max_input = 1.1 # the maximum magnitude of controller input, used to find a conversion between input and output
        self.thrust_factor = (max_output - min_output) / (2 * self.max_input) # conversion factor between input and output
        self.midpoint = (max_output + min_output) / 2

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2 # constant we found in matrix math stuff

        self.linear_factor = 1 # units (kg/s) arbitrary conversion between linear velocity and thrust, determined experimentally
        self.angular_factor = 0.32 if in_sim else 1 # units (kgm^2/s) arbitrary conversion between angular velocity and thrust, determined experimentally
       
        self.pid_omega = PID(0, 0, 0) # a pid constant for omega control
        self.pid_theta = PID(1, 0, 0) # a pid constant for theta control
        self.pid_x = PID(0, 0, 0)
        self.pid_y = PID(0, 0, 0)
        self.pid_vx = PID(0.5, 0, 0)
        self.pid_vy = PID(0.5, 0, 0)

        self.angular_pid_switcher = PIDSwitcher(self.pid_omega, self.pid_theta)
        self.linear_x_pid_switcher = PIDSwitcher(self.pid_vx, self.pid_x)
        self.linear_y_pid_switcher = PIDSwitcher(self.pid_vy, self.pid_y)

        self.theta = 0

        self.max_angular_velocity = 1 # custom constraint on theoretical angular velocity so we don't go out of control

        self.used_heading_last = False # whether or not the last input value was in theta mode
        self.last_update_timestamp = None # the last time the output loop ran
        self.last_data_timestamp = None # the last time we received data from odometry
        self.required_data_recentness = 1 # if we didn't get odometry data in the last X seconds, ignore data

        if in_sim: # set output ROS2 topic names
            self.front_left_name  = "/wamv/thrusters/front_left/thrust"
            self.front_right_name = "/wamv/thrusters/front_right/thrust"
            self.back_left_name   = "/wamv/thrusters/back_left/thrust"
            self.back_right_name  = "/wamv/thrusters/back_right/thrust"
        else:
            self.front_left_name  = "frontleft_pwm"
            self.front_right_name = "frontright_pwm"
            self.back_left_name   = "backleft_pwm"
            self.back_right_name  = "backright_pwm"

        self.all_thruster_names = (
            self.front_left_name,
            self.front_right_name,
            self.back_left_name,
            self.back_right_name
        )

        # subscriber for input to the controller
        self.create_subscription(
            ControlMessage, "/control_input", self.update_control_input, 10
        )
        #subscriber for data from the IMU
        self.create_subscription(
            Odometry, "/odometry/filtered", self.update_odometry, 10
        )
        # generate a publisher for each thruster
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                self.msg_type, thruster_prefix, 10
            )

        timer_period = 1/60 # update rate for the output loop
        self.timer = self.create_timer(timer_period, self.update_thrust) # start the output loop

    def get_thrust_values(self, tx, ty, tn):
        """
        Given linear and angular velocities, get thrust velocities for each thruster.

        Args:
            tx:
                The target x velocity of the boat
            ty:
                The target y velocity of the boat
            tn:
                The target angular velocity (omega) of the boat

        Returns:
            A dictionary mapping thruster topic name to output value, as a velocity
        """
        d = 2 ** (3 / 2)
        ang = tn * self.angular_factor / (4 * self.r)
        return {
            self.back_right_name: (tx - ty) * self.linear_factor / d + ang,
            self.back_left_name: (ty + tx) * self.linear_factor / d - ang,
            self.front_left_name: (tx - ty) * self.linear_factor / d - ang,
            self.front_right_name: (ty + tx) * self.linear_factor / d + ang,
        }
    
    def update_odometry(self, msg):
        """
        Callback function for when we receive data from the odometry
        """
        self.theta = R.from_quat([ # convert between quaternion and yaw value for theta
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ]).as_euler('xyz')[2]
        self.pid_theta.update_feedback_value(self.theta)
        self.pid_omega.update_feedback_value(msg.twist.twist.angular.z)

        self.pid_x.update_feedback_value(msg.pose.pose.position.x)
        self.pid_y.update_feedback_value(msg.pose.pose.position.y)
        self.pid_vx.update_feedback_value(msg.twist.twist.linear.x)
        self.pid_vy.update_feedback_value(msg.twist.twist.linear.y)

        self.last_data_timestamp = time.time() # keep a timestamp for the IMU update

    def update_thrust(self):
        """
        The main update function that sends output to the thrusters
        """
        current_time = time.time()
        if self.last_update_timestamp is not None:
            dt = current_time - self.last_update_timestamp
            x_input = 0
            y_input = 0
            angular_input = 0
            
            if self.last_data_timestamp is not None and current_time - self.last_data_timestamp <= self.required_data_recentness:
                print("theta: ", self.theta)
                x_input = self.linear_x_pid_switcher.determine_output(dt)
                y_input = self.linear_y_pid_switcher.determine_output(dt)
                angular_input = self.angular_pid_switcher.determine_output(dt)
            transformed_x = x_input * math.cos(self.theta) + y_input * math.sin(self.theta)
            transformed_y = y_input * math.cos(self.theta) - x_input * math.sin(self.theta)
            # print(('%.3f' %transformed_x), ('%.3f' %transformed_y), ('%.3f' %angular_input))
            results = self.get_thrust_values(x_input, y_input, angular_input)

            for name in self.all_thruster_names:
                # for each thruster:
                float_msg = self.msg_type()
                float_msg.data = self.py_type(self.restrict_input(
                    results[name], self.max_input # make sure the input isn't way out of bounds
                ) * self.thrust_factor + self.midpoint) # convert thrust to PWM
                self.thrust_publishers[name].publish(float_msg) # publish to thrusters
        self.last_update_timestamp = current_time # update timestamp for next time

    def restrict_input(self, input, max_val):
        """
        Helper function to ensure a given value is within a certain magnitude.

        Args:
            input:
                The input value
            max_val:
                The maximum magnitude of the output

        Returns:
            input if it's in the right range, or the bound that it's closest to if it's out of bounds
        """
        if input < -max_val:
            return -max_val
        if input > max_val:
            return max_val
        return input
    
    def about_zero(self, val):
        """
        Helper function to check whether a value is close to 0 (deadband).

        Args:
            val:
                The input value

        Returns:
            True if the value is about zero, False otherwise
        """
        return abs(val) < 0.001

    def update_control_input(self, msg):
        """
        Callback function that runs when we receive new input from the user
        """
        self.linear_x_pid_switcher.update_mode(msg.use_x_velocity)
        self.linear_x_pid_switcher.update_input(msg.x)

        self.linear_y_pid_switcher.update_mode(msg.use_y_velocity)
        self.linear_y_pid_switcher.update_input(msg.y)

        self.angular_pid_switcher.update_mode(msg.use_angular_velocity)
        self.angular_pid_switcher.update_input(msg.angular)



def main(args=None):
    rclpy.init(args=args)

    # initialize the controller
    controller = Controller()

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
