#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSConverter(Node):
    def __init__(self):
        super().__init__("gps_converter")
        self.nav_sat_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self.nav_sat_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.publisher_navsatfix = self.create_publisher(NavSatFix, "gps/fix", 10)

    def nav_sat_callback(self, msg: NavSatFix):
        # MAVROS sends STATUS_NO_FIX when RTK is not running (even if the data is "good").
        # Robot localization doesn't function when STATUS_NO_FIX, so we set status to STATUS_FIX
        msg.status.status = NavSatStatus.STATUS_FIX
        self.publisher_navsatfix.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gps_converter = GPSConverter()
    rclpy.spin(gps_converter)
    gps_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
