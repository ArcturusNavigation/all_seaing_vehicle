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

class Printer(Node):
    """
    A simple controller for x-drive. Receives velocities and/or heading as input
    and gives PWM output to thrusters.
    """

    def __init__(self):
        
        super().__init__("angle_printer")  # initialize ROS2

        
        self.create_subscription(Imu, "/mavros/imu/data_raw", self.update_imu, QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT, depth = 10))
        
    
    def update_imu(self, msg):
        """
        Callback function for when we receive only IMU data (this doesn't get called if we're using odometry).
        """
        self.update_theta(msg.orientation)

    def update_theta(self, orientation_msg):
        print(R.from_quat([ # convert between quaternion and yaw value for theta
            orientation_msg.x, orientation_msg.y, orientation_msg.z, orientation_msg.w
        ]).as_euler('xyz')[2])

def main(args=None):
    rclpy.init(args=args)

    # initialize the controller
    printer = Printer()
    rclpy.spin(printer)


if __name__ == "__main__":
    main()
