#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
import geopy.distance
from pyproj import CRS, Transformer

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")

        self.declare_parameter("global_frame_id", "odom")
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("datum", [42.3567949, -71.1070491, 0.0])
        self.declare_parameter("magnetic_declination", 0.0)
        self.declare_parameter("yaw_offset", np.pi/2.0) # angle wrt to east-aligned
        self.declare_parameter("odom_yaw_offset", np.pi/2.0)
        self.declare_parameter("odom_hz", 30.0)
        self.declare_parameter("use_odom_pos", False)
        self.declare_parameter("use_odom_yaw", False)
        self.declare_parameter("utm_zone", 19)
        self.declare_parameter("publish_tf", True)

        self.global_frame_id = self.get_parameter("global_frame_id").get_parameter_value().string_value
        self.base_link_frame = self.get_parameter("base_link_frame").get_parameter_value().string_value
        self.datum = self.get_parameter("datum").get_parameter_value().double_array_value
        self.odom_hz = self.get_parameter("odom_hz").get_parameter_value().double_value
        self.magnetic_declination = self.get_parameter("magnetic_declination").get_parameter_value().double_value
        self.yaw_offset = self.get_parameter("yaw_offset").get_parameter_value().double_value
        self.odom_yaw_offset = self.get_parameter("odom_yaw_offset").get_parameter_value().double_value
        self.use_odom_pos = self.get_parameter("use_odom_pos").get_parameter_value().bool_value
        self.use_odom_yaw = self.get_parameter("use_odom_yaw").get_parameter_value().bool_value
        self.utm_zone = self.get_parameter("utm_zone").get_parameter_value().integer_value
        self.publish_tf = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.datum_lat = self.datum[0]
        self.datum_lon = self.datum[1]
        self.datum_heading = self.datum[2] # angle of datum x axis wrt east

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
        self.yaw_odom_sub = self.create_subscription(
            Odometry,
            "yaw_odom_topic",
            self.yaw_odom_callback,
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
        self.got_yaw_odom = False

        self.odom_timer = self.create_timer(1.0/self.odom_hz, self.filter_cb)
        
        self.odom_pub = self.create_publisher(Odometry, "odometry/gps", 10)
        self.odom_integrated_pub = self.create_publisher(Odometry, "odometry/integrated", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_omega = 0.0
        self.last_odom_time = 0.0

    def nav_sat_callback(self, gps_msg):
        self.got_gps = True
        self.gps_msg = gps_msg
    
    def pos_odom_callback(self, pos_odom_msg):
        self.got_pos_odom = True
        self.pos_odom_msg = pos_odom_msg

    def yaw_odom_callback(self, yaw_odom_msg):
        self.got_yaw_odom = True
        self.yaw_odom_msg = yaw_odom_msg

    def compute_transform_from_to(self, from_pos, to_pos):
        from_x, from_y, from_theta = from_pos
        to_x, to_y, to_theta = to_pos
        dx = np.cos(from_theta)*(to_x-from_x)+np.sin(from_theta)*(to_y-from_y)
        dy = -np.sin(from_theta)*(to_x-from_x)+np.cos(from_theta)*(to_y-from_y)
        dtheta = to_theta - from_theta
        return (dx, dy, dtheta)

    def compose_transforms(self, t1, t2):
        t1_dx, t1_dy, t1_dtheta = t1
        t2_dx, t2_dy, t2_dtheta = t2
        t_dx = t1_dx+np.cos(t1_dtheta)*t2_dx-np.sin(t1_dtheta)*t2_dy
        t_dy = t1_dy+np.sin(t1_dtheta)*t2_dx+np.cos(t1_dtheta)*t2_dy
        t_dtheta = t1_dtheta+t2_dtheta
        return (t_dx, t_dy, t_dtheta)
    
    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg
        imu_twist = self.odom_msg.twist.twist

        self.vx = imu_twist.linear.x*np.cos(self.odom_yaw_offset) + imu_twist.linear.y*np.sin(self.odom_yaw_offset)
        self.vy = -imu_twist.linear.x*np.sin(self.odom_yaw_offset) + imu_twist.linear.y*np.cos(self.odom_yaw_offset)
        self.omega = imu_twist.angular.z

        if self.got_odom:
            # trapezoidal rule
            vx = 0.5*(self.vx+self.last_vx)
            vy = 0.5*(self.vy+self.last_vy)
            omega = 0.5*(self.omega+self.last_omega)
            dt = self.odom_msg.header.stamp.sec + self.odom_msg.header.stamp.nanosec*10**-9 - self.last_odom_time

            if omega < 0.001:
                dx = vx*dt
                dy = vy*dt
                dtheta = omega*dt
            else:
                dx = (np.sin(omega*dt)/omega)*vx+((np.cos(omega*dt)-1)/omega)*vy
                dy = ((1-np.cos(omega*dt))/omega)*vx+(np.sin(omega*dt)/omega)*vy
                dtheta = omega*dt

            self.odom_x, self.odom_y, self.odom_theta = self.compose_transforms((self.odom_x, self.odom_y, self.odom_theta), (dx, dy, dtheta))

        integrated_odom_msg = Odometry()
        integrated_odom_msg.header.stamp = self.odom_msg.header.stamp
        integrated_odom_msg.header.frame_id = self.global_frame_id
        integrated_odom_msg.child_frame_id = self.base_link_frame
        integrated_odom_msg.twist.twist.linear.x = self.vx
        integrated_odom_msg.twist.twist.linear.y = self.vy
        integrated_odom_msg.twist.twist.angular.z = self.omega
        integrated_odom_msg.pose.pose.position.x = self.odom_x
        integrated_odom_msg.pose.pose.position.y = self.odom_y
        integrated_odom_msg.pose.pose.orientation.x, integrated_odom_msg.pose.pose.orientation.y, integrated_odom_msg.pose.pose.orientation.z, integrated_odom_msg.pose.pose.orientation.w = quaternion_from_euler(0,0,self.odom_theta)
        self.odom_integrated_pub.publish(integrated_odom_msg)

        self.got_odom = True
        self.last_vx = self.vx
        self.last_vy = self.vy
        self.last_omega = self.omega
        self.last_odom_time = self.odom_msg.header.stamp.sec + self.odom_msg.header.stamp.nanosec*10**-9

    def filter_cb(self):
        if (not self.got_odom) or (not self.use_odom_pos and not self.got_gps) or (self.use_odom_pos and not self.got_pos_odom) or (self.use_odom_yaw and not self.got_pos_odom):
            return
        # get filtered values -> in imu_link -> rotated 90 degrees left wrt base_link
        # stamp = self.odom_msg.header.stamp
        # frame_id = self.odom_msg.header.frame_id
        if not self.use_odom_pos:
            lat, lon = self.gps_msg.latitude, self.gps_msg.longitude
        if not self.use_odom_yaw:
            roll,pitch,yaw = euler_from_quaternion([self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w])
        else:
            roll,pitch,yaw = euler_from_quaternion([self.yaw_odom_msg.pose.pose.orientation.x, self.yaw_odom_msg.pose.pose.orientation.y, self.yaw_odom_msg.pose.pose.orientation.z, self.yaw_odom_msg.pose.pose.orientation.w])
        # self.get_logger().info(f'RPY: {roll, pitch, yaw}')
        imu_heading = yaw
        actual_heading = imu_heading + self.yaw_offset - self.datum_heading
        imu_twist = self.odom_msg.twist.twist

        if(not self.use_odom_pos):
            # convert gps lat/lon to reference frame coordinates
            crs_latlon = CRS("EPSG:4326")
            crs_xy = CRS(f"EPSG:326{self.utm_zone}")
            transformer = Transformer.from_crs(crs_latlon, crs_xy, always_xy=True)
            x_boat, y_boat = transformer.transform(lon, lat)
            x_datum, y_datum = transformer.transform(self.datum_lon, self.datum_lat)
            dx = x_boat - x_datum
            dy = y_boat - y_datum
            # convert to rotated frame
            rotated_dx = np.cos(self.datum_heading)*dx + np.sin(self.datum_heading)*dy
            rotated_dy = -np.sin(self.datum_heading)*dx + np.cos(self.datum_heading)*dy
            dx = rotated_dx
            dy = rotated_dy
        else:
            dx = self.pos_odom_msg.pose.pose.position.x
            dy = self.pos_odom_msg.pose.pose.position.y
        
        # publish odometry (altitude is 0, we don't care about it)
        gps_odom_msg = Odometry()
        gps_odom_msg.header.stamp = self.get_clock().now().to_msg()
        gps_odom_msg.header.frame_id = self.global_frame_id
        gps_odom_msg.child_frame_id = self.base_link_frame
        gps_odom_msg.twist.twist.linear.x = imu_twist.linear.x*np.cos(self.odom_yaw_offset) + imu_twist.linear.y*np.sin(self.odom_yaw_offset)
        gps_odom_msg.twist.twist.linear.y = -imu_twist.linear.x*np.sin(self.odom_yaw_offset) + imu_twist.linear.y*np.cos(self.odom_yaw_offset)
        gps_odom_msg.twist.twist.angular = imu_twist.angular
        gps_odom_msg.pose.pose.position.x = dx
        gps_odom_msg.pose.pose.position.y = dy
        gps_odom_msg.pose.pose.orientation.x, gps_odom_msg.pose.pose.orientation.y, gps_odom_msg.pose.pose.orientation.z, gps_odom_msg.pose.pose.orientation.w = quaternion_from_euler(0,0,actual_heading)
        self.odom_pub.publish(gps_odom_msg)

        if self.publish_tf:
            # publish tf
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
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