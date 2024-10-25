#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
from mavros_msgs.msg import GPSRAW 
import math

class GPSRawConverter(Node):
    def __init__(self):
        super().__init__('gpsraw_converter')
        self.subscription = self.create_subscription(GPSRAW, "test_topic", self.gpsraw_callback, 10)
        self.publisher_navsatfix = self.create_publisher(NavSatFix, 'gps_navsatfix', 10)
        self.publisher_imu = self.create_publisher(Imu, 'gps_imu', 10)

    def gpsraw_to_navsatfix(self, gpsraw_msg):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = gpsraw_msg.header
        navsatfix_msg.latitude = gpsraw_msg.lat / 1e7  # Convert to degrees
        navsatfix_msg.longitude = gpsraw_msg.lon / 1e7  # Convert to degrees
        navsatfix_msg.altitude = gpsraw_msg.alt / 1000.0  # Convert to meters
        navsatfix_msg.position_covariance = [
            float(gpsraw_msg.eph**2), 0.0, 0.0,
            0.0, float(gpsraw_msg.epv**2), 0.0,
            0.0, 0.0, float(gpsraw_msg.eph**2)  # Horizontal accuracy
        ]
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        return navsatfix_msg
        
    def gpsraw_to_imu(self, gpsraw_msg):
        imu_msg = Imu()
        imu_msg.header = gpsraw_msg.header
        imu_msg.linear_acceleration.x = gpsraw_msg.vel / 1000.0  # Convert overall velocity to m/s²
        imu_msg.linear_acceleration.y = 0.0  # You may want to adjust this based on available data
        imu_msg.linear_acceleration.z = 0.0  # Adjust accordingly if you have vertical velocity data
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.orientation.z = math.sin(math.radians(gpsraw_msg.yaw)/2)
        imu_msg.orientation.w = math.cos(math.radians(gpsraw_msg.yaw)/2)
        imu_msg.orientation_covariance[0] = -1  # Indicate no orientation data
        return imu_msg

    def gpsraw_callback(self, gpsraw_msg):
        navsatfix_msg = self.gpsraw_to_navsatfix(gpsraw_msg)
        self.publisher_navsatfix.publish(navsatfix_msg)
        imu_msg = self.gpsraw_to_imu(gpsraw_msg)
        self.publisher_imu.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    gpsraw_subscriber = GPSRawConverter()
    try:
        rclpy.spin(gpsraw_subscriber)
    except KeyboardInterrupt:
        pass
    gpsraw_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()