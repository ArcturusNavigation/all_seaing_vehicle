#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from all_seaing_interfaces.msg import ControlMessage

# easy way to publish messages to /control_input with given parameters
# use --ros-args to specify arguments
# use vx:=123 to set x velocity relative to boat
# use vy:=123 to set y velocity relative to boat
# use angular:=123 to set angular parameter
# use use_heading:=True to tell the controller whether to use heading or velocity control on angle


class SampleControlPublisher(Node):

    def __init__(self):
        super().__init__('sample_control_publisher')
        self.publisher_ = self.create_publisher(ControlMessage, 'control_input', 10)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        
        self.declare_parameter('angular', 0.0)

        self.declare_parameter('use_linear_velocity', True)
        self.declare_parameter('use_angular_velocity', True)

        self.msg = ControlMessage()
        self.msg.x = float(self.get_parameter('x').value)
        self.msg.y = float(self.get_parameter('y').value)
        self.msg.angular = float(self.get_parameter('angular').value)
        self.msg.linear_control_mode = ControlMessage.LOCAL_VELOCITY if bool(self.get_parameter('use_linear_velocity').value) else ControlMessage.WORLD_POSITION
        self.msg.angular_control_mode = ControlMessage.WORLD_VELOCITY if bool(self.get_parameter('use_angular_velocity').value) else ControlMessage.WORLD_POSITION 

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.cb)

    def cb(self):
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    pub = SampleControlPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
