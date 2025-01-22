#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlOption

import serial
import struct
import time

class RoverLoraController(Node):
    def __init__(self):
        super().__init__('rover_lora_controller')

        # Setup serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        time.sleep(2)  # Allow serial port to stabilize
        
        self.data_size = struct.calcsize('BdddB')

        # Create ROS 2 publisher for ControlOption
        self.control_publisher = self.create_publisher(ControlOption, 'control_options', 10)

        # Start a ROS 2 timer to check for serial data
        self.timer = self.create_timer(0.01, self.check_serial_data)
        
        
    def calculate_checksum(self, data):
        return sum(data) % 256

    def check_serial_data(self):
        if self.serial_port.in_waiting > 0:
            # Read serialized JSON data from the serial port
            serialized_data = self.serial_port.read(self.data_size)
            
            if len(serialized_data) != self.data_size:
                return
        
            priority, x, y, angular, received_checksum = struct.unpack('BdddB', serialized_data)
            
            # Verify checksum
            data_without_checksum = serialized_data[:-1]
            calculated_checksum = self.calculate_checksum(data_without_checksum)
            
            if calculated_checksum != received_checksum:
                print("Checksum mismatch, discarding data")
                return
                        
            control_msg = ControlOption()
            control_msg.priority = priority
            control_msg.twist.linear.x = x
            control_msg.twist.linear.y = y
            control_msg.twist.angular.z = angular
            self.control_publisher.publish(control_msg)
            


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
