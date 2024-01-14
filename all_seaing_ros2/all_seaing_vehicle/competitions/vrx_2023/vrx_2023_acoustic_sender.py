#!/usr/bin/env python3
import rclpy

from geometry_msgs.msg import Point
from all_seaing_vehicle.competitions.vrx_task_node import TaskNode
from protobuf_client_interfaces.msg import Gateway

class AcousticWPTSender(TaskNode):

    def __init__(self):
        super().__init__("acoustic_wpt_sender")
        self.subscription = self.create_subscription(
            Point,
            "/pinger_coord",
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)

    def listener_callback(self, msg: Point):
        waypt_msg = Gateway()
        waypt_msg.gateway_key = "WPT_UPDATE"
        waypt_msg.gateway_string = f"points={msg.x},{msg.y}"
        if self.task_name in ("acoustic_tracking", "acoustic_perception"):
            self.publisher.publish(waypt_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AcousticWPTSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
