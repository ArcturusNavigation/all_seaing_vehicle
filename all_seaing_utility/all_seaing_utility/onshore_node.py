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

        self.control_message = ControlMessage()
        self.control_input_publisher = self.create_publisher(
            ControlMessage, "/control_input", 10
        )
        self.control_message.linear_control_mode = ControlMessage.LOCAL_VELOCITY
        self.control_message.angular_control_mode = ControlMessage.WORLD_VELOCITY

        self.heartbeat_message = Heartbeat()
        self.heartbeat_publisher = self.create_publisher(Heartbeat, "/heartbeat", 10)
        self.heartbeat_message.in_teleop = True
        self.heartbeat_message.e_stopped = False

        # Setup subscriber
        self.joy_control_sub = self.create_subscription(
            Joy, "joy", self.keyboard_callback, 10
        )

        self.heartbeat_timer = self.create_timer(
            HEART_RATE, self.beat_heart
        )  # start the output loop

        self.enter_held = False

        self.get_logger().info("starting onshore node, teleop enabled")

    def beat_heart(self):
        self.heartbeat_publisher.publish(self.heartbeat_message)

    def keyboard_callback(self, msg):
        if self.heartbeat_message.e_stopped:
            self.get_logger().fatal("we are e-stopped!")
            return
        if msg.buttons[0]:
            self.get_logger().info("e-stop pressed!")
            self.heartbeat_message.e_stopped = True
            return
        if msg.buttons[1]:
            if not self.enter_held:
                self.enter_held = True
                self.heartbeat_message.in_teleop = not self.heartbeat_message.in_teleop
                self.get_logger().info(
                    f"toggled teleop (now {self.heartbeat_message.in_teleop})"
                )
        elif self.enter_held:
            self.enter_held = False
        if self.heartbeat_message.in_teleop:
            self.control_message.y = msg.axes[0] * -0.6
            self.control_message.x = msg.axes[1] * 0.6
            self.control_message.angular = msg.axes[2] * -0.3
            self.control_input_publisher.publish(self.control_message)


def main(args=None):
    # Start node, and spin
    rclpy.init(args=args)
    node = OnshoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     # Clean up and shutdown
    #     node.destroy_node()
    #     rclpy.shutdown()


if __name__ == "__main__":
    main()
