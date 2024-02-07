#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

class SimTimeFixer(Node):

    def __init__(self):
        super().__init__('sim_time_fixer')
        self.create_subscription(
            Imu, "/wamv/sensors/imu/imu/data", self.make_update(
                self.create_publisher(Imu, '/imu/data', 10)
            ), 10
        )
        self.create_subscription(
            NavSatFix, "/wamv/sensors/gps/gps/fix", self.make_update(
                self.create_publisher(NavSatFix, '/gps/fix', 10)
            ), 10
        )

    def make_update(self, publisher):
        def update(data):
            t = time.time()
            data.header.stamp.sec = int(t)
            data.header.stamp.nanosec = int((t % 1)*(10**9))
            publisher.publish(data)
        return update

def main(args=None):
    rclpy.init(args=args)
    pub = SimTimeFixer()
    rclpy.spin(pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
