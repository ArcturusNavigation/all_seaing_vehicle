#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
from queue import Queue
import serial
import struct
import time
from all_seaing_common.report_pb2 import *

class RoverLoraReporter(Node):
    def __init__(self):
        super().__init__('rover_lora_controller')

        port = self.declare_parameter(
            "port", "/dev/ttyACM0").get_parameter_value().string_value
        
        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.report_queue = Queue()

        if not self.is_sim:

            # Setup serial connection
            self.serial_port = serial.Serial(port, 115200, timeout=1)
            time.sleep(2)  # Allow serial port to stabilize

        self.reporter_subscriber = self.create_subscription(ByteMultiArray, "task_reporter", self.add_message, 10)

        self.timer = self.create_timer(0.2, self.send_message_if_exists)
        
    def send_message_if_exists(self):
        # TODO we need to process heartbeat in a separate queue and publish it as it arrives, to not have it get dropped/put back in the queue
        if not self.report_queue.empty():
            raw_msg = self.report_queue.get()
            checksum = sum(raw_msg) % 256
            frame = b'$R' + struct.pack("!B", len(raw_msg)) + struct.pack("!B", checksum) + raw_msg + b'!!'
            if not self.is_sim:
                self.serial_port.write(frame)
                self.serial_port.flush()
            else:
                msg = Report()
                msg.ParseFromString(raw_msg)
                self.get_logger().info(msg.__str__())
        
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
