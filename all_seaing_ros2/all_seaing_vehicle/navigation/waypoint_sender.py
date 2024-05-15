#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from protobuf_client_interfaces.msg import Gateway


class WaypointSender(Node):

    def __init__(self):
        super().__init__("waypoint_sender")

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            String,
            "/waypoints",
            self.gateway_cb,
            10,
        )
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)
    
    def gateway_cb(self, msg):
        wpt_msg = Gateway()
        if msg.gateway_key in ("WAYPOINT_UPDATE", "DEPLOY", "MOOS_MANUAL_OVERRIDE"):
            wpt_msg.gateway_key = msg.gateway_key
            wpt_msg.gateway_string = msg.gateway_string
        self.publisher.publish(wpt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
