#!/usr/bin/env python3

# Does not build.
# Do not use this node as a dependency.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from all_seaing_interfaces.msg import ASVState


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Scaling thrust/rudder
        self.linear_scaling = self.declare_parameter(
            "linear_scaling", 25.0).get_parameter_value().double_value
        self.angular_scaling = self.declare_parameter(
            "angular_scaling", 15.0).get_parameter_value().double_value
        self.lower_thrust_limit = self.declare_parameter(
            "lower_thrust_limit", -1400.0).get_parameter_value().double_value
        self.upper_thrust_limit = self.declare_parameter(
            "upper_thrust_limit", 1400.0).get_parameter_value().double_value

        # Publishers and subscribers
        self.left_thrust_pub = self.create_publisher(
            Float64, "thrusters/left/thrust", 10
        )
        self.right_thrust_pub = self.create_publisher(
            Float64, "thrusters/right/thrust", 10
        )
        self.command_sub = self.create_subscription(
            ASVState, "asv_state", self.command_callback, 10
        )

    def command_callback(self, msg):
        left_thrust_msg = Float64()
        right_thrust_msg = Float64()

        # Calculate left and right thrust values
        left_thrust = msg.desired_thrust * self.linear_scaling
        right_thrust = msg.desired_thrust * self.linear_scaling
        left_thrust += msg.desired_rudder * self.angular_scaling
        right_thrust -= msg.desired_rudder * self.angular_scaling

        # Limit outputs
        left_thrust_msg.data = max(
            self.lower_thrust_limit, min(left_thrust, self.upper_thrust_limit)
        )
        right_thrust_msg.data = max(
            self.lower_thrust_limit, min(right_thrust, self.upper_thrust_limit)
        )

        # Publish left and right thrust values
        self.left_thrust_pub.publish(left_thrust_msg)
        self.right_thrust_pub.publish(right_thrust_msg)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
