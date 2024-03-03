#!/usr/bin/env python3

import abc

import rclpy
from rclpy import qos
from rclpy.node import Node

from sensor_msgs.msg import Joy
from keyboard_msgs.msg import Key

# neccessary to publish controlmessages
from all_seaing_interfaces.msg import ControlMessage

import yaml


class JoyPart(abc.ABC):
    def __init__(self, init_value):
        self._value = init_value

    @abc.abstractmethod
    def down(self, code):
        pass

    @abc.abstractmethod
    def up(self, code):
        pass

    def get(self):
        return self._value


class Button(JoyPart):
    def __init__(self, key_str):
        super().__init__(0)
        self.code = getattr(Key, key_str)

    def down(self, code):
        if code == self.code:
            self._value = 1

    def up(self, code):
        if code == self.code:
            self._value = 0


class Axis(JoyPart):
    def __init__(self, key_neg_str, key_pos_str):
        super().__init__(0.0)
        self.code_neg = getattr(Key, key_neg_str)
        self.code_pos = getattr(Key, key_pos_str)

    def down(self, code):
        if code == self.code_neg:
            self._value -= 1.0

        elif code == self.code_pos:
            self._value += 1.0

    def up(self, code):
        if code == self.code_neg:
            self._value += 1.0

        elif code == self.code_pos:
            self._value -= 1.0


class JoyToControlNode(Node):
    def __init__(self):
        # Initialize ROS node
        super().__init__("joy_to_control_node")


        # Setup publisher
        self.m_control = ControlMessage()
        self.joy_control_pub = self.create_publisher(ControlMessage, '/control_input', 10)


        # Setting up controller
        self.m_control.use_x_velocity = True
        self.m_control.use_y_velocity = True
        self.m_control.use_angular_velocity = True

        # Setup subscriber
        self.joy_control_sub = self.create_subscription(Joy, "joy", self.vel_callback, 10)
        self.m_joy = Joy()


    def vel_callback(self, msg):
        self.m_joy = msg
        self.m_control.y = self.m_joy.axes[0] * .2
        self.m_control.x = self.m_joy.axes[1] * .2
        self.m_control.angular = self.m_joy.axes[2] * .3
        self.joy_control_pub.publish(self.m_control)




def main(args=None):
    # Start node, and spin
    rclpy.init(args=args)
    node = JoyToControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up and shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
