#!/usr/bin/env python3
import rclpy
import rclpy.time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import Int64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from all_seaing_interfaces.msg import Heartbeat
from all_seaing_interfaces.msg import ControlMessage

# WHETHER TO NOT USE ODOMETRY i.e. ARE WE INSIDE?? 
BLIND_LINEAR = False

class DoNothingPID:
    def determine_output(self, _, __):
        return 0
    def update_feedback_value(self, _):
        pass
    def reset():
        pass

class PID:
    """
    A data class representing a PID object.
    """
    def __init__(self, pid_tuple, get_feedback_value, debug_name = None, default_to_input = False):
        """
        Initialize the PID with constant p, i, and d values. 
        If debug_name is not none, we will print the current error value whenever the pid is updated. 
        default_value represents the defualt output value for the PID when no feedback has been received yet.
        """
        self.p = pid_tuple[0]
        self.i = pid_tuple[1]
        self.d = pid_tuple[2]
        self.feedback = None
        self.default_to_input = default_to_input
        self.accumulated_error = 0
        self.previousError = None
        self.debug_name = debug_name
        self.get_feedback_value = get_feedback_value
    def update_feedback_value(self, msg):
        """
        Update the feedback value for this PID controller.
        """
        self.feedback = self.get_feedback_value(msg)
    def get_error(self, input):
        """
        Get the error from the feedback given an input. This is separated into a function so that it can be overridden.
        """
        return input - self.feedback
    def determine_output(self, input, dt):
        """
        Given an input and a delta-time, use PID to compute the output value.
        """
        if self.feedback is None:
            return input if self.default_to_input else 0
        error = self.get_error(input)
        if self.debug_name is not None:
            print(self.debug_name + " error: " + str(error))
        self.accumulated_error += error * dt
        derror_dt = 0 if self.previousError is None else (error - self.previousError)/dt
        self.previousError = error
        return self.p * error + self.i * self.accumulated_error + self.d * derror_dt
    def reset(self):
        """
        Reset the PID so that I and D stored values are reset.
        """
        self.accumulated_error = 0
        self.previousError = None

class CircularPID(PID):
    """
    Represents a PID where input/error is mod 2(pi).
    """
    def get_error(self, input):
        diff = (input - self.feedback) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        return diff

# x = forward
# y = left
class Controller(Node):
    """
    A simple controller for x-drive. Receives velocities and/or heading as input
    and gives PWM output to thrusters.
    """

    def __init__(self):
        """
        Initialize the controller.
        """
        super().__init__("xdrive_controller")  # initialize ROS2

        self.declare_parameter("in_sim", False)
        self.in_sim = bool(self.get_parameter("in_sim").value)
        print("in sim: ", self.in_sim)

        l = 3.5 if self.in_sim else 0.7112  # BOAT LENGTH
        w = 2 if self.in_sim else 0.2540  # BOAT WIDTH
        self.msg_type = Float64 if self.in_sim else Int64
        self.py_type = float if self.in_sim else int
        min_output = (
            -1000 if self.in_sim else 1100
        )  # the minimum PWM output value for thrusters
        max_output = (
            1000 if self.in_sim else 1900
        )  # the maximum PWM output value for thrusters
        self.max_input = 1.1 # the maximum magnitude of controller input, used to find a conversion between input and output
        self.thrust_factor = (max_output - min_output) / (
            2 * self.max_input
        )  # conversion factor between input and output
        self.midpoint = (max_output + min_output) / 2

        self.r = (
            (l**2 + w**2) / 2 - l * w
        ) ** 0.5 / 2  # constant we found in matrix math stuff
        
        pid_omega = PID((1, 0.5, 0) if self.in_sim else (0.0, 0.00, 0), self.get_omega_feedback)#0.8, 0.04, 0
        pid_theta = CircularPID((1, 0, 0) if self.in_sim else (0.0, 0, 0), self.get_theta_feedback)

        pid_x = PID((0.1, 0, 0) if self.in_sim else (0.0, 0, 0), self.get_x_position_feedback, None)
        pid_y = PID((0.1, 0, 0) if self.in_sim else (0.0, 0, 0), self.get_y_position_feedback, None)
        pid_vx = PID((0.4, 0.1, 0) if self.in_sim else (0, 0, 0), self.get_x_velocity_world_feedback, None, BLIND_LINEAR)
        pid_vy = PID((0.4, 0.1, 0) if self.in_sim else (0, 0, 0), self.get_y_velocity_world_feedback, None, BLIND_LINEAR)
        local_pid_vx = PID((0.4, 0.1, 0) if self.in_sim else (1, 0, 0), self.get_x_velocity_local_feedback, None, BLIND_LINEAR)
        local_pid_vy = PID((0.4, 0.1, 0) if self.in_sim else (0.0, 0, 0), self.get_y_velocity_local_feedback, None, BLIND_LINEAR)

        do_nothing_pid = DoNothingPID()

        self.x_control_handlers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_x,
            ControlMessage.WORLD_VELOCITY: pid_vx,
            ControlMessage.LOCAL_VELOCITY: local_pid_vx
        }

        self.y_control_handlers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_y,
            ControlMessage.WORLD_VELOCITY: pid_vy,
            ControlMessage.LOCAL_VELOCITY: local_pid_vy
        }

        self.angular_control_handlers = {
            ControlMessage.OFF: do_nothing_pid,
            ControlMessage.WORLD_POSITION: pid_theta,
            ControlMessage.WORLD_VELOCITY: pid_omega,
            ControlMessage.LOCAL_VELOCITY: pid_omega
        }

        self.chosen_angular_control_mode = ControlMessage.OFF
        self.chosen_linear_control_mode = ControlMessage.OFF

        self.angular_input = 0
        self.x_input = 0
        self.y_input = 0

        self.theta = 0

        self.last_update_timestamp = None # the last time the output loop ran
        self.last_data_timestamp = None # the last time we received data from odometry
        self.required_data_recentness = 1 # if we didn't get odometry data in the last X seconds, ignore data
        self.last_heartbeat_timestamp = self.get_time()
        self.required_heartbeat_recentness = 3

        if self.in_sim:  # set output ROS2 topic names
            self.front_left_name = "/wamv/thrusters/front_left/thrust"
            self.front_right_name = "/wamv/thrusters/front_right/thrust"
            self.back_left_name = "/wamv/thrusters/back_left/thrust"
            self.back_right_name = "/wamv/thrusters/back_right/thrust"
        else:
            self.front_left_name = "/frontleft_pwm"
            self.front_right_name = "/frontright_pwm"
            self.back_left_name = "/backleft_pwm"
            self.back_right_name = "/backright_pwm"

        self.all_thruster_names = (
            self.front_left_name,
            self.front_right_name,
            self.back_left_name,
            self.back_right_name,
        )

        # subscriber for input to the controller
        self.create_subscription(
            ControlMessage, "/control_input", self.update_control_input, 10
        )
        self.create_subscription(
            Heartbeat, "/heartbeat", self.receive_heartbeat, 10
        )
        #subscriber for data from the IMU
        if BLIND_LINEAR:
            imu_topic_name = "/wamv/sensors/imu/imu/data" if self.in_sim else "/mavros/imu/data"
            if self.in_sim:
                self.create_subscription(Imu, imu_topic_name, self.update_imu, 10)
            else:
                self.create_subscription(Imu, imu_topic_name, self.update_imu, QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, depth = 10))
        else:
            self.create_subscription(Odometry, "/odometry/filtered", self.update_odometry, 10)
        
        # generate a publisher for each thruster
        self.thrust_publishers = {}
        for thruster_prefix in self.all_thruster_names:
            self.thrust_publishers[thruster_prefix] = self.create_publisher(
                self.msg_type, thruster_prefix, 10
            )

        timer_period = 1/60 if self.in_sim else 1/8  # update rate for the output loop
        self.timer = self.create_timer(
            timer_period, self.update_thrust
        )  # start the output loop

    def get_omega_feedback(self, msg):
        return msg.angular_velocity.z if BLIND_LINEAR else msg.twist.twist.angular.z
    def get_theta_feedback(self, _):
        print(self.theta)
        return self.theta
    def get_x_position_feedback(self, msg):
        return msg.pose.pose.position.x
    def get_y_position_feedback(self, msg):
        return msg.pose.pose.position.y
    def get_x_velocity_local_feedback(self, msg):
        #print(f"feedback: {msg.twist.twist.linear.x}")
        return msg.twist.twist.linear.x
    def get_y_velocity_local_feedback(self, msg):
        return msg.twist.twist.linear.y
    def get_x_velocity_world_feedback(self, msg):
        return msg.twist.twist.linear.x * math.cos(self.theta) - msg.twist.twist.linear.y * math.sin(self.theta)
    def get_y_velocity_world_feedback(self, msg):
        return msg.twist.twist.linear.y * math.cos(self.theta) + msg.twist.twist.linear.x * math.sin(self.theta)
    def do_nothing_feedback(self, _):
        return 0

    def get_time(self):
        """
        Get the current time. Written into a function because it's annoying to write every time.
        """
        return self.get_clock().now()

    def receive_heartbeat(self, msg):
        """
        Callback function for receiving hearbeat messages, which are necessary for the controller to run.
        """
        if msg.e_stopped:
            self.convert_to_pwm_and_send(self.get_thrust_values(0, 0, 0))
            raise Exception("received e-stop message! killing node")
        self.last_heartbeat_timestamp = self.get_time()

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
        ang = tn / (4 * self.r)
        return {
            self.back_right_name: (tx - ty) / d + ang,
            self.back_left_name: (ty + tx) / d - ang,
            self.front_left_name: (1 if self.in_sim else -1) * ((tx - ty) / d - ang),
            self.front_right_name: (1 if self.in_sim else -1) * ((ty + tx) / d + ang),
        }
    
    def update_imu(self, msg):
        """
        Callback function for when we receive only IMU data (this doesn't get called if we're using odometry).
        """
        self.update_theta(msg, msg.orientation)
    
    def update_odometry(self, msg):
        """
        Callback function for when we receive data from the odometry.
        """
        self.update_theta(msg, msg.pose.pose.orientation)
        #print("updating x feedback")
        self.x_control_handlers[self.chosen_linear_control_mode].update_feedback_value(msg)
        self.y_control_handlers[self.chosen_linear_control_mode].update_feedback_value(msg)

    def update_theta(self, msg, orientation_msg):
        self.theta = R.from_quat([ # convert between quaternion and yaw value for theta
            orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w
        ]).as_euler('xyz')[2] 
        
        self.angular_control_handlers[self.chosen_angular_control_mode].update_feedback_value(msg)

        self.last_data_timestamp = self.get_time()

    def time_diff(self, a, b):
        """
        Subtract two rclpy Times and convert to seconds, floating-point.
        """
        return (a - b).nanoseconds / 1e9
    
    def convert_to_pwm_and_send(self, thrust_values):
        """
        Given a thrust value dictionary, convert the values to PWM and send to the respective publishers.
        """
        for name in self.all_thruster_names:
            # for each thruster:
            float_msg = self.msg_type()
            float_msg.data = self.py_type(self.restrict_input(
                thrust_values[name], self.max_input # make sure the input isn't way out of bounds
            ) * self.thrust_factor + self.midpoint) # convert thrust to PWM
            #print(float_msg)
            self.thrust_publishers[name].publish(float_msg) # publish to thrusters

    def update_thrust(self):
        """
        The main update function that sends output to the thrusters
        """
        #print("updating")
        current_time = self.get_time()
        #print(current_time)
        #print("heartbeat time: ")
        #print(self.last_heartbeat_timestamp)
        if self.time_diff(current_time, self.last_heartbeat_timestamp) > self.required_heartbeat_recentness:
            self.convert_to_pwm_and_send(self.get_thrust_values(0, 0, 0))
            raise Exception("lost heartbeat! killing node")
        if self.last_update_timestamp is not None:
            dt = self.time_diff(current_time, self.last_update_timestamp)
            x_output = 0
            y_output = 0
            angular_output = 0
            
            if self.last_data_timestamp is not None and self.time_diff(current_time, self.last_data_timestamp) <= self.required_data_recentness:
                x_output = self.x_control_handlers[self.chosen_linear_control_mode].determine_output(self.x_input, dt)
                y_output = self.y_control_handlers[self.chosen_linear_control_mode].determine_output(self.y_input, dt)
                angular_output = self.angular_control_handlers[self.chosen_angular_control_mode].determine_output(self.angular_input, dt)

            # print("raw pid output: ", (x_output, y_output))

            if self.chosen_linear_control_mode == ControlMessage.WORLD_VELOCITY or self.chosen_linear_control_mode == ControlMessage.WORLD_POSITION:
                x_boat_space = x_output * math.cos(self.theta) + y_output * math.sin(self.theta)
                y_boat_space = y_output * math.cos(self.theta) - x_output * math.sin(self.theta)
            else:
                x_boat_space = x_output
                y_boat_space = y_output

            # print("boat: ", (x_boat_space, y_boat_space))
            results = self.get_thrust_values(x_boat_space, y_boat_space, angular_output)
            # print(results)

            self.convert_to_pwm_and_send(results)
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

    def update_control_input(self, msg):
        """
        Callback function that runs when we receive new input from the user
        """
        new_chosen_linear_control_mode = msg.linear_control_mode
        new_chosen_angular_control_mode = msg.angular_control_mode

        if new_chosen_linear_control_mode != self.chosen_linear_control_mode:
            self.x_control_handlers[new_chosen_linear_control_mode].reset()
            self.y_control_handlers[new_chosen_linear_control_mode].reset()
            self.chosen_linear_control_mode = new_chosen_linear_control_mode

        if new_chosen_angular_control_mode != self.chosen_angular_control_mode:
            self.angular_control_handlers[new_chosen_angular_control_mode].reset()
            self.chosen_angular_control_mode = new_chosen_angular_control_mode

        self.x_input = msg.x
        self.y_input = msg.y
        self.angular_input = msg.angular

def main(args=None):
    rclpy.init(args=args)

    # initialize the controller
    controller = Controller()
    rclpy.spin(controller)


if __name__ == "__main__":
    main()
