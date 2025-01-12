#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import random

class WayPointPublisherTest(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_test')

        # Create publishers for waypoints and markers
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'waypoint_markers', 10)

        timer_period = 5  # Publish every 5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Ensure the frame matches RViz setup

        marker_array = MarkerArray()

        # Generate 2 random waypoints
        for i in range(2):
            pose = Pose()
            pose.position.x = float(random.uniform(0, 10))
            pose.position.y = float(random.uniform(0, 10))

            # Add pose to PoseArray
            msg.poses.append(pose)

            # Create a marker for each waypoint
            marker = Marker()
            marker.header = msg.header
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0  # Fully opaque
            marker.color.r = 1.0  # Red

            marker_array.markers.append(marker)

        # Publish waypoints and markers
        self.waypoints_publisher.publish(msg)
        self.marker_publisher.publish(marker_array)

        self.get_logger().info('Published waypoints and markers')

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WayPointPublisherTest()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
