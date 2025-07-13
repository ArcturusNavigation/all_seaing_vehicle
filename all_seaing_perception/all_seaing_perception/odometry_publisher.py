#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import geopy.distance

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")

        self.declare_parameter("datum", [42.3567949, -71.1070491, 0.0])
        self.declare_parameter("odom_hz", 30.0)

        self.datum = self.get_parameter("datum").get_parameter_value().double_array_value
        self.odom_hz = self.get_parameter("odom_hz").get_parameter_value().double_value
        self.datum_lat = self.datum[0]
        self.datum_lon = self.datum[1]
        self.datum_heading = self.datum[2] # actual heading of imu's 0 (- imu's value when facing east)

        self.nav_sat_sub = self.create_subscription(
            NavSatFix,
            "gps_topic",
            self.nav_sat_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom_topic",
            self.odom_callback,
            10,
        )

        self.got_gps = False
        self.got_odom = False

        self.odom_timer = self.create_timer(1.0/self.odom_hz, self.filter_cb)
        
        self.odom_pub = self.create_publisher(Odometry, "odometry/gps", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def nav_sat_callback(self, gps_msg):
        self.got_gps = True
        self.gps_msg = gps_msg
    
    def odom_callback(self, odom_msg):
        self.got_odom = True
        self.odom_msg = odom_msg
    def filter_cb(self):
        if (not self.got_odom) or (not self.got_gps):
            return
        # get filtered values -> in imu_link -> rotated 90 degrees left wrt base_link
        stamp = self.gps_msg.header.stamp
        frame_id = self.gps_msg.header.frame_id
        lat, lon = self.gps_msg.latitude, self.gps_msg.longitude
        _,_,imu_heading = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])
        actual_heading = imu_heading
        imu_twist = self.odom_msg.twist.twist

        # convert gps lat/lon to reference frame coordinates
        # adapted from tag_transformer.py
        delta = geopy.distance.geodesic((self.datum_lat, self.datum_lon), (lat, lon)).meters
        heading = np.arctan2(lon-self.datum_lon, lat-self.datum_lat)
        dx = delta * np.cos(heading-self.datum_heading)
        dy = delta * np.sin(heading-self.datum_heading)
        
        # publish odometry (altitude is 0, we don't care about it)
        gps_odom_msg = Odometry()
        gps_odom_msg.header.stamp = stamp
        gps_odom_msg.header.frame_id = "odom"
        gps_odom_msg.child_frame_id = "base_link" # same as imu_link here
        gps_odom_msg.twist.twist.linear.x = imu_twist.linear.x
        gps_odom_msg.twist.twist.linear.y = imu_twist.linear.y
        gps_odom_msg.pose.pose.position.x = dx
        gps_odom_msg.pose.pose.position.y = dy
        gps_odom_msg.pose.pose.orientation.x, gps_odom_msg.pose.pose.orientation.y, gps_odom_msg.pose.pose.orientation.z, gps_odom_msg.pose.pose.orientation.w = quaternion_from_euler(0,0,actual_heading)
        self.odom_pub.publish(gps_odom_msg)

        # publish tf
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = dx
        tf_msg.transform.translation.y = dy
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = quaternion_from_euler(0,0,actual_heading)
        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()