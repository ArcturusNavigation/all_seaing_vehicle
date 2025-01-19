#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class VelocityStats(Node):
    def __init__(self):
        super().__init__('velocity_stats')
        self.group = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )
        
        self.max_vel_pub = self.create_publisher(Float64, "max_vel", 10)
        self.mean_vel_pub = self.create_publisher(Float64, "mean_vel", 10)
        
        self.max_vel = 0
        self.vel_sum = 0
        self.vel_count = 0

    def odom_callback(self, msg: Odometry):
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        
        self.v_mag = (self.vx**2 + self.vy**2)**0.5
        
        if self.v_mag > 0.05:
            self.vel_sum += self.v_mag
            self.vel_count += 1
            
            self.max_vel = max(self.max_vel, self.v_mag)
        
        self.max_vel_pub.publish(Float64(data=self.max_vel))
        self.mean_vel_pub.publish(Float64(data=self.vel_sum / self.vel_count))


def main(args=None):
    rclpy.init(args=args)
    velocity_stats = VelocityStats()
    rclpy.spin(velocity_stats)
    velocity_stats.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()