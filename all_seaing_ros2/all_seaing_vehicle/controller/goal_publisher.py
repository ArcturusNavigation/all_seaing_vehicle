#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from all_seaing_interfaces.msg import GoalState

# easy way to publish messages to /control_input with given parameters
# use --ros-args to specify arguments
# use vx:=123 to set x velocity relative to boat
# use vy:=123 to set y velocity relative to boat
# use angular:=123 to set angular parameter
# use use_heading:=True to tell the controller whether to use heading or velocity control on angle

class goal_publisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(GoalState, '/goal_state', 10)
        # self.declare_parameter('goal_lat', 0.0)
        # self.declare_parameter('goal_lon', 0.0)
        # self.declare_parameter('goal_heading', 0.0)

        self.msg = GoalState()
        #self.message.goal_lat = float(self.get_parameter('goal_lat').value)
        #self.message.goal_lon = float(self.get_parameter('goal_lon').value)
        #self.message.goal_heading = float(self.get_parameter('goal_heading').value)

        self.msg.goal_lat = float( 0.00003)
        self.msg.goal_lon = float( 0.00003)
        self.msg.goal_heading = float(1.7)

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.goal_callback)
    
    def goal_callback(self):
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    node = goal_publisher()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
