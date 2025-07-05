#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

class OdomReframe(Node):
    def __init__(self):
        super().__init__("odom_reframe")
        self.declare_parameter("target_child_frame_id", "base_link")
        self.target_child_frame_id = self.get_parameter("target_child_frame_id").get_parameter_value().string_value

        self.odom_sub = self.create_subscription(Odometry, "odom_topic", self.odom_cb, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "new_odom_topic", qos_profile_sensor_data)

    def odom_cb(self, odom_msg):
        odom_msg.child_frame_id = self.target_child_frame_id
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_reframe = OdomReframe()
    rclpy.spin(odom_reframe)
    odom_reframe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()