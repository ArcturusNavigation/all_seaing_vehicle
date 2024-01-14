#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from ros_gz_interfaces.msg import ParamVec
from protobuf_client_interfaces.msg import Gateway

class TaskSender(Node):

    def __init__(self):
        super().__init__("task_sender")
        self.subscription = self.create_subscription(
            ParamVec,
            "/vrx/task/info",
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Gateway, "/send_to_gateway", 10)

    def listener_callback(self, msg):
        task_name_msg = Gateway()
        task_name = "null"
        for elem in msg.params:
            if elem.name == "name":
                task_name = elem.value.string_value

        task_name_msg.gateway_key = "TASK_NAME"
        task_name_msg.gateway_string = task_name
        self.publisher.publish(task_name_msg)

        deploy_msg = Gateway()
        override_msg = Gateway()
        deploy = False
        for elem in msg.params:
            if elem.name == "state":
                deploy = elem.value.string_value in ("ready", "running")
        deploy_msg.gateway_key = "DEPLOY"
        deploy_msg.gateway_string = str(deploy).lower()
        override_msg.gateway_key = "MOOS_MANUAL_OVERRIDE"
        override_msg.gateway_string = str(not deploy).lower()
        self.publisher.publish(deploy_msg)
        self.publisher.publish(override_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TaskSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
