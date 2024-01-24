#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header


class EStoppedPublisherTest(Node):

    def __init__(self):
        super().__init__('e_stopped_publisher_test')
        self.publisher_ = self.create_publisher(Header, 'heartbeat', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.stamp.sec)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = EStoppedPublisherTest()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()