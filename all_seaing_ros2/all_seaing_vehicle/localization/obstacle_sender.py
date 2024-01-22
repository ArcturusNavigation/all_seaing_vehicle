#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleFeature
from protobuf_client_interfaces.msg import Gateway

class ObstacleSender(Node):

    def __init__(self):
        super().__init__("obstacle_sender")

        # True if using GPS, False if using local UTM
        self.declare_parameter("use_gps", True)
        self.use_gps = bool(self.get_parameter("use_gps").value)

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            ObstacleFeature,
            "/tracked_feature",
            self.obstacle_cb,
            10)
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)

    def obstacle_cb(self, msg):
        feature_msg = Gateway()
        feature_msg.gateway_key = "TRACKED_FEATURE_GPS" if self.use_gps else "TRACKED_FEATURE"
        feature_msg.gateway_string = f"x={msg.x},y={msg.y},label={msg.label}"
        self.publisher.publish(feature_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
