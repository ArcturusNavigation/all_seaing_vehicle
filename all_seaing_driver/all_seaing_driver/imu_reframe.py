#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

class ImuReframe(Node):
    def __init__(self):
        super().__init__("imu_reframe")
        self.declare_parameter("target_frame_id", "base_link")
        self.target_frame_id = self.get_parameter("target_frame_id").get_parameter_value().string_value

        self.imu_sub = self.create_subscription(Imu, "imu_topic", self.imu_cb, qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "new_imu_topic", qos_profile_sensor_data)

    def imu_cb(self, imu_msg):
        imu_msg.header.frame_id = self.target_frame_id
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_reframe = ImuReframe()
    rclpy.spin(imu_reframe)
    imu_reframe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()