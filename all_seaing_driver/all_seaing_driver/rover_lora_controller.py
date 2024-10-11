#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlMessage
import serial
import struct
import time

class RoverLoraController(Node):
    def __init__(self):
        super().__init__('rover_lora_controller')

        # Setup serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        time.sleep(2)  # Allow serial port to stabilize
        
        self.data_size = struct.calcsize('BdddBB')

        # Create ROS 2 publisher for ControlMessage
        self.publisher_ = self.create_publisher(ControlMessage, 'control_options', 10)

        # Start a ROS 2 timer to check for serial data
        self.timer = self.create_timer(0.1, self.check_serial_data)

    def check_serial_data(self):
        if self.serial_port.in_waiting > 0:
            # Read serialized JSON data from the serial port
            serialized_data = self.serial_port.read(self.data_size)
            
            if len(serialized_data) != self.data_size:
                return
            
            priority, x, y, angular, linear_control_mode, angular_control_mode = struct.unpack('BdddBB', serialized_data)

            # Create a ControlMessage ROS 2 message
            control_msg = ControlMessage()
            control_msg.priority = priority
            control_msg.x = x
            control_msg.y = y
            control_msg.angular = angular
            control_msg.linear_control_mode = linear_control_mode
            control_msg.angular_control_mode = angular_control_mode
            # Publish the ControlMessage
            self.publisher_.publish(control_msg)
            self.get_logger().info(f"Published: {control_msg}")

def main(args=None):
    rclpy.init(args=args)
    serial_control_subscriber = RoverLoraController()

    try:
        rclpy.spin(serial_control_subscriber)
    except KeyboardInterrupt:
        pass

    # Close serial connection
    serial_control_subscriber.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
