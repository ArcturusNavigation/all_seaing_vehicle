#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class EStoppedNode(Node):

    def __init__(self, name = "default_e_stopped_node"):
        super().__init__(name)

        self.declare_parameter('timeout_duration', 5)
        self.timeout_duration = self.get_parameter('timeout_duration').get_parameter_value().integer_value

        self.subscription = self.create_subscription(Header, 'heartbeat', self.listener_callback, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_received_time = self.get_clock().now().to_msg().sec
        self.is_e_stopped = False

    def timer_callback(self):
        if self.get_clock().now().to_msg().sec - self.last_received_time > self.timeout_duration:
            self.get_logger().info(f'Node is e_stopped: Has not received in the last {self.timeout_duration} seconds since time: {self.last_received_time}')
            self.is_e_stopped = True

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%d"' % msg.stamp.sec)
        self.last_received_time = msg.stamp.sec


def main(args=None):
    rclpy.init(args=args)

    subscriber = EStoppedNode()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
