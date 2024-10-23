#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Header
from my_package.msg import GPSRAW  # Import your custom GPSRAW message

class GPSRawConverter(Node):
    def __init__(self):
        super().__init__('gpsraw_converter')
        self.subscription = self.create_subscription(GPSRAW, "test_topic", self.gpsraw_callback, 10)
        # change test topic to actual gps 

        self.publisher_natsavfix = self.create_publisher(NavSatFix, 'gps_natsatfix', 10)
        self.publisher_imu = self.create_publisher(Imu, 'gps_imu', 10)

    def gpsraw_to_navsatfix(self, gpsraw_msg):
        navsatfix_msg = NavSatFix()
        # Fill in the NavSatFix message
        navsatfix_msg.header = gpsraw_msg.header
        navsatfix_msg.latitude = gpsraw_msg.lat / 1e7  # Convert to degrees
        navsatfix_msg.longitude = gpsraw_msg.lon / 1e7  # Convert to degrees
        navsatfix_msg.altitude = gpsraw_msg.alt / 1000.0  # Convert to meters
        # Set status (example: 3D fix or no fix)
        navsatfix_msg.status.status = NavSatFix.STATUS_FIX if gpsraw_msg.fix_type >= 3 else NavSatFix.STATUS_NO_FIX
        navsatfix_msg.status.service = NavSatFix.SERVICE_GPS
        # Set covariance based on position accuracy (example)
        navsatfix_msg.position_covariance = [
            gpsraw_msg.eph**2, 0, 0,
            0, gpsraw_msg.epv**2, 0,
            0, 0, gpsraw_msg.epv**2
        ]
        navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        return navsatfix_msg
        
    def gpsraw_to_imu(self, gpsraw_msg):
        imu_msg = Imu()
        # Fill in IMU message data (linear acceleration based on velocity)
        imu_msg.header = gpsraw_msg.header
        imu_msg.linear_acceleration.x = gpsraw_msg.vel_e / 1000.0  # Convert to m/s²
        imu_msg.linear_acceleration.y = gpsraw_msg.vel_n / 1000.0  # Convert to m/s²
        imu_msg.linear_acceleration.z = gpsraw_msg.vel_d / 1000.0  # Convert to m/s²
        # Assuming no angular velocity and orientation available from GPS
        imu_msg.angular_velocity.x = 0
        imu_msg.angular_velocity.y = 0
        imu_msg.angular_velocity.z = 0
        imu_msg.orientation_covariance[0] = -1  # Indicate no orientation data
        return imu_msg

    def gpsraw_callback(self, gpsraw_msg):
        # Convert GPSRAW to NavSatFix
        navsatfix_msg = self.gpsraw_to_navsatfix(gpsraw_msg)
        self.publisher_natsavfix.publish(navsatfix_msg)
        # Convert GPSRAW to Imu
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
