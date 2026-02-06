#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from queue import Queue
import serial
import struct
import time

class RoverLoraReporter(Node):
    def __init__(self):
        super().__init__('rover_lora_controller')

        self.report_queue = Queue()

        # Setup serial connection
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # Allow serial port to stabilize

        self.reporter_subscriber = self.create_subscription(ByteMultiArray, "task_reporter", self.add_message, 10)

        self.timer = self.create_timer(0.2, self.send_message_if_exists)
        
    def send_message_if_exists(self):
        if not self.report_queue.empty():
            raw_msg = self.report_queue.get()
            frame = struct.pack("!I", len(raw_msg)) + raw_msg + "test".encode()
            self.serial_port.write(frame)
            self.serial_port.flush()
        
    def add_message(self, msg: ByteMultiArray):
        self.report_queue.put(b"".join(msg.data))


def main(args=None):
    rclpy.init(args=args)
    rover_reporter = RoverLoraReporter()
    rclpy.spin(rover_reporter)
    rover_reporter.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
