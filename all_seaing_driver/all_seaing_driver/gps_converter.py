#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from mavros_msgs.msg import GPSRAW


class GPSConverter(Node):
    def __init__(self):
        super().__init__("gps_converter")

        self.yaw_offset = self.declare_parameter(
            "yaw_offset", 180.0).get_parameter_value().double_value

        self.nav_sat_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.nav_sat_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.gpsraw_sub = self.create_subscription(
            GPSRAW, "/mavros/gpsstatus/gps2/raw", self.gpsraw_callback, 10
        )
        self.publisher_navsatfix = self.create_publisher(NavSatFix, "gps/fix", 10)
        self.imu_pub = self.create_publisher(Imu, 'gps/imu', 10)

    def nav_sat_callback(self, msg: NavSatFix):
        # MAVROS sends STATUS_NO_FIX when RTK is not running (even if the data is "good").
        # Robot localization doesn't function when STATUS_NO_FIX, so we set status to STATUS_FIX
        msg.status.status = NavSatStatus.STATUS_FIX
        self.publisher_navsatfix.publish(msg)

    def gpsraw_callback(self, gpsraw_msg):
        imu_msg = Imu()
        imu_msg.header = gpsraw_msg.header
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.z = math.sin(math.radians(-gpsraw_msg.yaw / 100 + self.yaw_offset) / 2)
        imu_msg.orientation.w = math.cos(math.radians(-gpsraw_msg.yaw / 100 + self.yaw_offset) / 2)
        if gpsraw_msg.yaw != 65535:
            self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_converter = GPSConverter()
    rclpy.spin(gps_converter)
    gps_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
