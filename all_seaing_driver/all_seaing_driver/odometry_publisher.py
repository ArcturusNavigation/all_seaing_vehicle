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

        self.declare_parameter("global_frame_id", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("datum", [42.3567949, -71.1070491, 0.0])
        self.declare_parameter("magnetic_declination", 0.0)
        self.declare_parameter("yaw_offset", -np.pi/2.0)
        self.declare_parameter("odom_yaw_offset", np.pi/2.0)
        self.declare_parameter("odom_hz", 30.0)
        self.declare_parameter("use_odom_pos", False)

        self.global_frame_id = self.get_parameter("global_frame_id").get_parameter_value().string_value
        self.base_link_frame = self.get_parameter("base_link_frame").get_parameter_value().string_value
        self.datum = self.get_parameter("datum").get_parameter_value().double_array_value
        self.odom_hz = self.get_parameter("odom_hz").get_parameter_value().double_value
        self.magnetic_declination = self.get_parameter("magnetic_declination").get_parameter_value().double_value
        self.yaw_offset = self.get_parameter("yaw_offset").get_parameter_value().double_value
        self.odom_yaw_offset = self.get_parameter("odom_yaw_offset").get_parameter_value().double_value
        self.use_odom_pos = self.get_parameter("use_odom_pos").get_parameter_value().bool_value
        self.datum_lat = self.datum[0]
        self.datum_lon = self.datum[1]
        self.datum_heading = self.datum[2] # actual heading of imu's 0 (- imu's value when facing east)

        self.nav_sat_sub = self.create_subscription(
            NavSatFix,
            "gps_topic",
            self.nav_sat_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.pos_odom_sub = self.create_subscription(
            Odometry,
            "pos_odom_topic",
            self.pos_odom_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom_topic",
            self.odom_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.got_gps = False
        self.got_pos_odom = False
        self.got_odom = False

        self.odom_timer = self.create_timer(1.0/self.odom_hz, self.filter_cb)
        
        self.odom_pub = self.create_publisher(Odometry, "odometry/gps", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def nav_sat_callback(self, gps_msg):
        self.got_gps = True
        self.gps_msg = gps_msg
    
    def pos_odom_callback(self, pos_odom_msg):
        self.got_pos_odom = True
        self.pos_odom_msg = pos_odom_msg
    
    def odom_callback(self, odom_msg):
        self.got_odom = True
        self.odom_msg = odom_msg
    def filter_cb(self):
        if (not self.got_odom) or (not self.use_odom_pos and not self.got_gps) or (self.use_odom_pos and not self.got_pos_odom):
            return
        # get filtered values -> in imu_link -> rotated 90 degrees left wrt base_link
        stamp = self.odom_msg.header.stamp
        frame_id = self.odom_msg.header.frame_id
        if not self.use_odom_pos:
            lat, lon = self.gps_msg.latitude, self.gps_msg.longitude
        _,_,imu_heading = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])
        actual_heading = imu_heading + self.yaw_offset
        imu_twist = self.odom_msg.twist.twist

        if(not self.use_odom_pos):
            # convert gps lat/lon to reference frame coordinates
            delta = geopy.distance.geodesic((self.datum_lat, self.datum_lon), (lat, lon)).meters
            heading = np.arctan2(lon-self.datum_lon, lat-self.datum_lat)
            # TODO: hotfix or real? should be swapped
            dy = delta * np.cos(heading-self.magnetic_declination-self.datum_heading)
            dx = delta * np.sin(heading-self.magnetic_declination-self.datum_heading)
        else:
            dx = self.pos_odom_msg.pose.pose.position.x
            dy = self.pos_odom_msg.pose.pose.position.y
        
        # publish odometry (altitude is 0, we don't care about it)
        gps_odom_msg = Odometry()
        gps_odom_msg.header.stamp = stamp
        gps_odom_msg.header.frame_id = self.global_frame_id
        gps_odom_msg.child_frame_id = self.base_link_frame
        gps_odom_msg.twist.twist.linear.x = imu_twist.linear.x*np.cos(self.odom_yaw_offset) + imu_twist.linear.y*np.sin(self.odom_yaw_offset)
        gps_odom_msg.twist.twist.linear.y = -imu_twist.linear.x*np.sin(self.odom_yaw_offset) + imu_twist.linear.y*np.cos(self.odom_yaw_offset)
        gps_odom_msg.twist.twist.angular = imu_twist.angular
        gps_odom_msg.pose.pose.position.x = dx
        gps_odom_msg.pose.pose.position.y = dy
        gps_odom_msg.pose.pose.orientation.x, gps_odom_msg.pose.pose.orientation.y, gps_odom_msg.pose.pose.orientation.z, gps_odom_msg.pose.pose.orientation.w = quaternion_from_euler(0,0,actual_heading)
        self.odom_pub.publish(gps_odom_msg)

        # publish tf
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.global_frame_id
        tf_msg.child_frame_id = self.base_link_frame
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