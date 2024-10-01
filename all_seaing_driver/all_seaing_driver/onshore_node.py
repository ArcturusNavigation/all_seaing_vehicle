#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from all_seaing_interfaces.msg import ControlMessage
from all_seaing_interfaces.msg import Heartbeat

HEART_RATE = 1


class OnshoreNode(Node):
    def __init__(self):
        super().__init__("onshore_node")

        self.declare_parameter("joy_x_scale", 2.0)
        self.declare_parameter("joy_y_scale", -2.0)
        self.declare_parameter("joy_ang_scale", -0.8)

        self.joy_x_scale = self.get_parameter("joy_x_scale").value
        self.joy_y_scale = self.get_parameter("joy_y_scale").value
        self.joy_ang_scale = self.get_parameter("joy_ang_scale").value

        self.heartbeat_message = Heartbeat()
        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeat", 10)
        self.heartbeat_message.in_teleop = True
        self.heartbeat_message.e_stopped = False

        self.enter_held = False

        self.control_input_publisher = self.create_publisher(
            ControlMessage, "control_options", 10
        )
        self.joy_control_sub = self.create_subscription(
            Joy, "/joy", self.keyboard_callback, 10
        )
        self.heartbeat_timer = self.create_timer(
            HEART_RATE, self.beat_heart
        )

        self.get_logger().info("Starting onshore node, teleop enabled")

    def beat_heart(self):
        self.heartbeat_publisher.publish(self.heartbeat_message)

    def send_controls(self, x, y, angular):
        control_message = ControlMessage()
        control_message.priority = 0
        control_message.linear_control_mode = ControlMessage.LOCAL_VELOCITY
        control_message.angular_control_mode = ControlMessage.WORLD_VELOCITY
        control_message.x = x
        control_message.y = y
        control_message.angular = angular
        self.control_input_publisher.publish(control_message)

    def keyboard_callback(self, msg):
        if self.heartbeat_message.e_stopped:
            self.get_logger().fatal("ASV is e-stopped!")
            return

        if msg.buttons[0]: # msg.buttons[0] = space bar --> e-stop (reference config)
            self.get_logger().info("E-stop pressed!")
            self.heartbeat_message.e_stopped = True
            self.heartbeat_publisher.publish(self.heartbeat_message)
            return

        if msg.buttons[1]: # msg.buttons[1] = return key (reference config)
            if not self.enter_held:
                self.enter_held = True
                self.heartbeat_message.in_teleop = not self.heartbeat_message.in_teleop
                self.heartbeat_publisher.publish(self.heartbeat_message)
                self.get_logger().info(
                    f"Toggled teleop (now {self.heartbeat_message.in_teleop})"
                )
        elif self.enter_held:
            self.enter_held = False

        if self.heartbeat_message.in_teleop:
            self.send_controls(
                msg.axes[1] * self.joy_x_scale,
                msg.axes[0] * self.joy_y_scale,
                msg.axes[2] * self.joy_ang_scale
            )

def main(args=None):
    rclpy.init(args=args)
    node = OnshoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
