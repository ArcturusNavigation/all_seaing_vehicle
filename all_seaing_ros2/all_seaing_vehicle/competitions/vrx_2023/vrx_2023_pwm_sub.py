#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PWMSubscriber(Node):
    def __init__(self):
        super().__init__('pwm_subscriber')
        self.num_topics = 4
        self.topic_names = ['back_left', 'back_right', 'front_left', 'front_right']
        assert len(self.topic_names) == self.num_topics
        self.reset_data()
        
        for i in range(self.num_topics):
            self.create_subscription(
                Float64,
                f'/wamv/thrusters/{self.topic_names[i]}/thrust',
                self.make_callback(i),
                10
            )

    def reset_data(self):
        self.received_data = [None] * self.num_topics
        self.num_received = 0

    def make_callback(self, index):
        def callback(data):
            if self.received_data[index] is None:
                self.num_received += 1
            self.received_data[index] = data.data
            if self.num_received >= self.num_topics:
                printing = ''
                for i in range(self.num_topics):
                    printing += f'{self.topic_names[i]}: {self.received_data[i]}\n'
                print(printing)
                print()
                self.reset_data()
        return callback

def main(args=None):
    rclpy.init(args=args)
    sub = PWMSubscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
