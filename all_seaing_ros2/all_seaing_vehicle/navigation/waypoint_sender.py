#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from protobuf_client_interfaces.msg import Gateway


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")

        # True is using PoseArray, False if using PoseStamped
        self.declare_parameter("use_pose_array", True)
        self.use_pose_array = bool(self.get_parameter("use_pose_array").value)

        # True if using GPS, False if using local UTM
        self.declare_parameter("use_gps", True)
        self.use_gps = bool(self.get_parameter("use_gps").value)

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            PoseArray if self.use_pose_array else PoseStamped,
            "/waypoints",
            self.wpt_cb,
            10,
        )
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)

    def wpt_cb(self, msg):

        # Set up in_msg based on different message types
        in_msg = PoseArray()
        if self.use_pose_array:
            in_msg = msg
        else:
            in_msg.header = msg.header
            in_msg.poses.append(msg.pose)

        # Parse waypoint(s) to send to MOOS
        wpt_msg = Gateway()
        inner_string = "" if self.use_gps else "points="
        for i, pose in enumerate(in_msg.poses):
            inner_string += f"{pose.position.x},{pose.position.y}"
            if i < len(in_msg.poses) - 1:
                inner_string += ":"
        wpt_msg.gateway_key = "WPT_UPDATE_GPS" if self.use_gps else "WPT_UPDATE"
        wpt_msg.gateway_string = inner_string
        self.publisher.publish(wpt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
