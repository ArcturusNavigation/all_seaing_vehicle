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

class Pub(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(GoalState, '/stationkeeping_input', 10)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_heading', 0.0)

        self.msg = GoalState()
        self.message.goal_x = float(self.get_parameter('goal_x').value)
        self.message.goal_y = float(self.get_parameter('goal_y').value)
        self.message.goal_heading = float(self.get_parameter('goal_heading').value)

        timer_period = 1/60  # seconds
        self.timer = self.create_timer(timer_period, self.cb)
    
    def cb(self):
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)

    pub = Pub()

    rclpy.spin(pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
