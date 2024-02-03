#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from all_seaing_interfaces.msg import ASVState

# easy way to publish messages to /control_input with given parameters
# use --ros-args to specify arguments
# use vx:=123 to set x velocity relative to boat
# use vy:=123 to set y velocity relative to boat
# use angular:=123 to set angular parameter
# use use_heading:=True to tell the controller whether to use heading or velocity control on angle


class Pub(Node):

    def __init__(self):
        super().__init__("state_publisher")
        self.publisher_ = self.create_publisher(ASVState, "/stationkeeping_input", 10)
        self.declare_parameter("nav_lat", 0.0)
        self.declare_parameter("nav_long", 0.0)
        self.declare_parameter("nav_x", 0.0)
        self.declare_parameter("nav_y", 0.0)
        self.declare_parameter("nav_heading", 0.0)
        self.declare_parameter("nav_speed", 0.0)

        self.msg = ASVState()
        self.msg.nav_lat = float(self.get_parameter("nav_lat").value)
        self.msg.nav_long = float(self.get_parameter("nav_long").value)
        self.msg.nav_x = float(self.get_parameter("nav_x").value)
        self.msg.nav_y = float(self.get_parameter("nav_y").value)
        self.msg.nav_heading = float(self.get_parameter("nav_heading").value)
        self.msg.nav_speed = float(self.get_parameter("nav_speed").value)

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.cb)

    def cb(self):
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    pub = Pub()

    rclpy.spin(pub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
