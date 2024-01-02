#!/usr/bin/env python3
import rclpy
import time
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu

from all_seaing_interfaces.msg import ControlMessage

class PID:
    """
    A data class representing a PID object.
    """
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d

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
        self.i = ControlMessage() # represents the most recent input value
        self.i.vx = 0.0 # x velocity
        self.i.vy = 0.0 # y velocity
        self.i.angular = 0.0 # angular parameter - theta if use_heading, otherwise omega
        self.i.use_heading = False # determines theta versus omega control 

        self.r = ((l ** 2 + w ** 2) / 2 - l * w) ** 0.5 / 2 # constant we found in matrix math stuff

        self.linear_factor = 1 # units (kg/s) arbitrary conversion between linear velocity and thrust, determined experimentally
        self.angular_factor = 0.32 if in_sim else 1 # units (kgm^2/s) arbitrary conversion between angular velocity and thrust, determined experimentally
       
        self.pid_omega = PID(10, 0, 0) # a pid constant for omega control
        self.pid_theta = PID(1.0, 0.0005, 0.5) # a pid constant for theta control

        self.max_angular_velocity = 1 # custom constraint on theoretical angular velocity so we don't go out of control

        self.actual_omega = 0 # most recent IMU value for omega
        self.actual_theta = 0 # most recent IMU value for theta (converted from quaternion)

        self.accumulated_error = 0 # generic accumulated error value (omega or theta) used for I control
        self.last_error = None # most recent error value (omega or theta) used for D control

        self.used_heading_last = False # whether or not the last input value was in theta mode
        self.last_update_timestamp = None # the last time the output loop ran
        self.last_imu_data_timestamp = None # the last time we received data from IMU
        self.required_imu_data_recentness = 1 # if we didn't get IMU data in the last X seconds, ignore data

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
            Imu, "/wamv/sensors/imu/imu/data", self.update_heading, 10
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
    
    def update_heading(self, msg):
        """
        Callback function for when we receive data from the IMU
        """
        self.actual_omega = msg.angular_velocity.z
        self.actual_theta = R.from_quat([ # convert between quaternion and yaw value for theta
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ]).as_euler('xyz')[2]
        self.last_imu_data_timestamp = time.time() # keep a timestamp for the IMU update

    def update_thrust(self):
        """
        The main update function that sends output to the thrusters
        """
        if self.debug_mode:
            print(self.actual_theta if self.i.use_heading else self.actual_omega)
        current_time = time.time()
        angular_input = 0 if self.i.use_heading else self.i.angular # this represents what we will pass in as angular velocity
        if self.last_imu_data_timestamp is not None and (current_time - self.last_imu_data_timestamp) <= self.required_imu_data_recentness:
            # if there's valid data from the IMU, run feedback stuff
            if self.i.use_heading:
                # if in theta mode, set PID accordingly and get error value
                pid = self.pid_theta
                diff = (self.i.angular - self.actual_theta) % (2 * math.pi)
                if diff > math.pi:
                    diff -= 2 * math.pi
            else:
                # otherwise, get error value using omega
                pid = self.pid_omega
                diff = self.i.angular - self.actual_omega
            angular_input += pid.p * diff # add P contribution to angular input

            if self.last_update_timestamp is not None:
                # if this isn't the first time updating, run the rest of the feedback
                dt = current_time - self.last_update_timestamp

                if self.about_zero(diff) or (self.accumulated_error > 0) != (diff > 0):
                    self.accumulated_error = 0 # optimization? for I control so it doesn't go crazy in the wrong direction
                self.accumulated_error += diff * dt # update accumulated error

                d_comp = 0 if self.last_error is None else (diff - self.last_error) / dt # calculate D contribution

                angular_input += pid.i * self.accumulated_error + pid.d * d_comp # add I and D contribution to angular input
            
            self.last_error = diff # update last error for future calculations

        # get thrust values using linear and angular velocity inputs
        results = self.get_thrust_values(self.i.vx, self.i.vy, self.restrict_input(angular_input, self.max_angular_velocity))

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
        self.i = msg
        if msg.use_heading != self.used_heading_last:
            self.accumulated_error = 0 # if changing modes from omega to theta or vice versa, reset feedback variables
            self.last_error = None
        self.used_heading_last = msg.use_heading

def main(args=None):
    rclpy.init(args=args)

    # initialize the controller
    controller = Controller()

    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
