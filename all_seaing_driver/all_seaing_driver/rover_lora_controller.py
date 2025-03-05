#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import (
    ControlOption, 
    Heartbeat, 
    KeyboardButton
)

import serial
import struct
import time

class RoverLoraController(Node):
    def __init__(self):
        super().__init__('rover_lora_controller')

        # Setup serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
        time.sleep(1)  # Allow serial port to stabilize
        
        # self.data_size = struct.calcsize('BdddBBB')
        # self.data_size = struct.calcsize('BdddBB')
        self.data_size = struct.calcsize('<H')

        # Create ROS 2 publisher for ControlOption
        self.control_publisher = self.create_publisher(ControlOption, 'control_options', 10)

        # Start a ROS 2 timer to check for serial data
        self.timer = self.create_timer(0.05, self.check_serial_data)
                
        self.heartbeat_publisher = self.create_publisher(Heartbeat, "heartbeat", 10)

        self.keyboard_publisher = self.create_publisher(KeyboardButton, "keyboard_button", 10)
        
        
    def calculate_checksum(self, data):
        return sum(data) % 256

    def check_serial_data(self):
        if self.serial_port.in_waiting > 0:
            # Read serialized JSON data from the serial port
            serialized_data = self.serial_port.read(self.data_size)
            line = self.serial_port.readline()
        
            val = struct.unpack('<H', serialized_data)
            val = val[0]
#            last7 = val % 128
#            if last7 != 127:
#                print("discarding value")
#                return
            self.get_logger().info(f"received value: {bin(val)}")

            cnt = 15
            control_keys = {}
            
            for i in range(6):
                let = "qweasd"
                control_keys[let[i]] = bool(val // (2 ** cnt) )
                cnt -= 1

            heartbeat_message = Heartbeat()
            heartbeat_message.in_teleop = bool(val // 2 ** cnt)
            cnt -= 1
            heartbeat_message.e_stopped = bool(val // 2 ** cnt)
            cnt -= 1
            self.heartbeat_publisher.publish(heartbeat_message)

            keyboard_msg = KeyboardButton()
            keyboard_msg.key = "p" if bool(val // 2 ** cnt) else "0"
            self.keyboard_publisher.publish(keyboard_msg)
            x = 0.0
            y = 0.0
            angular = 0.0

            if control_keys["a"]: y = 0.6
            elif control_keys["d"]: y = -0.6

            if control_keys["w"]: x = 1.0
            elif control_keys["s"]: x = -1.0

            if control_keys["q"]: angular = 0.3
            elif control_keys["e"]: angular = -0.3

            control_msg = ControlOption()
            control_msg.priority = 0
            control_msg.twist.linear.x = x
            control_msg.twist.linear.y = y
            control_msg.twist.angular.z = angular
            self.control_publisher.publish(control_msg)
            
           # heartbeat_message = Heartbeat()
           # heartbeat_message.in_teleop = bool(in_teleop)
           # heartbeat_message.e_stopped = bool(e_stopped)
           # self.heartbeat_publisher.publish(heartbeat_message)

           # keyboard_msg = KeyboardButton()
           # keyboard_msg.key = chr(key)
           # self.keyboard_publisher.publish(keyboard_msg)



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
