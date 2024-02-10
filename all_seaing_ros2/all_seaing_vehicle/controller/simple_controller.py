#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from all_seaing_interfaces.msg import ASVState


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # scaling MOOS thrust/rudder from -100:100
        self.declare_parameter("linear_scaling", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("angular_scaling", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("lower_thrust_limit", rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("upper_thrust_limit", rclpy.Parameter.Type.DOUBLE)
        self.linear_scaling = self.get_parameter("linear_scaling").value
        self.angular_scaling = self.get_parameter("angular_scaling").value
        self.lower_thrust_limit = self.get_parameter("lower_thrust_limit").value
        self.upper_thrust_limit = self.get_parameter("upper_thrust_limit").value

        # publishers (remap left_thrust and right_thrust in launch file to appropriate topics)
        self.left_thrust_pub = self.create_publisher(Float64, "left_thrust", 10)
        self.right_thrust_pub = self.create_publisher(Float64, "right_thrust", 10)

        # subscribers
        self.command_sub = self.create_subscription(
            ASVState, "/asv_state", self.command_callback, 10
        )

    def command_callback(self, msg):
        left_thrust_msg = Float64()
        right_thrust_msg = Float64()

        # calculate left and right thrust values
        left_thrust = msg.desired_thrust * self.linear_scaling
        right_thrust = msg.desired_thrust * self.linear_scaling
        left_thrust += msg.desired_rudder * self.angular_scaling
        right_thrust -= msg.desired_rudder * self.angular_scaling

        # limit outputs
        left_thrust_msg.data = max(
            self.lower_thrust_limit, min(left_thrust, self.upper_thrust_limit)
        )
        right_thrust_msg.data = max(
            self.lower_thrust_limit, min(right_thrust, self.upper_thrust_limit)
        )

        # publish left and right thrust values
        self.left_thrust_pub.publish(left_thrust_msg)
        self.right_thrust_pub.publish(right_thrust_msg)


if __name__ == "__main__":
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
