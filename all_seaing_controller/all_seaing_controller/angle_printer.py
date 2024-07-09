#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu


class AnglePrinter(Node):
    """
    A simple controller for x-drive. Receives velocities and/or heading as input
    and gives PWM output to thrusters.
    """

    def __init__(self):

        super().__init__("angle_printer")

        self.create_subscription(
            Imu,
            "/mavros/imu/data_raw",
            self.update_imu,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10),
        )

    def update_imu(self, msg):
        """
        Callback function for when we receive only IMU data 
        (this doesn't get called if we're using odometry).
        """
        self.update_theta(msg.orientation)

    def update_theta(self, orientation_msg):
        # Convert between quaternion and yaw value for theta
        rpy = R.from_quat(
            [
                orientation_msg.x,
                orientation_msg.y,
                orientation_msg.z,
                orientation_msg.w,
            ]
        ).as_euler("xyz")
        self.get_logger().info(f"Roll: {rpy[0]}, pitch: {rpy[1]}, yaw: {rpy[2]}")


def main(args=None):
    rclpy.init(args=args)
    angle_printer = AnglePrinter()
    rclpy.spin(angle_printer)
    angle_printer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
