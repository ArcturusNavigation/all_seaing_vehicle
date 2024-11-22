#!/usr/bin/env python3

import pygame
import struct
import serial
import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_controller')
        
        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 400))
        pygame.display.set_caption("Keyboard Controller")
        self.font = pygame.font.Font(None, 36)
        
        # determine if simulation or real robot
        self.declare_parameter('is_real', False)
        self.is_real = self.get_parameter('is_real').get_parameter_value().bool_value

        if self.is_real:
            # Serial setup
            self.get_logger().info("Connecting to serial port...")
            self.serial_port = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)
            time.sleep(2)  # Allow serial port to stabilize
            self.get_logger().info("Connected!")
        else:
            # Initialize ROS publisher
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 50 ms timer

    def calculate_checksum(self, data):
        return sum(data) % 256

    def publish_cmd_vel(self, control_msg):
        twist = Twist()
        twist.linear.x = control_msg["x"]
        twist.linear.y = control_msg["y"]
        twist.angular.z = control_msg["angular"]
        self.cmd_vel_pub.publish(twist)

    def control_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                pygame.quit()
                sys.exit(0)

        keys = pygame.key.get_pressed()
        
        # Control message format
        control_msg = {
            "priority": 0,
            "x": 0.0,
            "y": 0.0,
            "angular": 0.0,
        }
        
        # Adjust control message based on arrow keys
        if keys[pygame.K_a]:
            control_msg["y"] = 1.0  # Move left
        elif keys[pygame.K_d]:
            control_msg["y"] = -1.0  # Move right
        
        if keys[pygame.K_w]:
            control_msg["x"] = is_sim ? 5.0 else 1.0  # Move forward
        elif keys[pygame.K_s]:
            control_msg["x"] = is_sim ? -5.0 else -1.0  # Move backward
            
        if keys[pygame.K_q]:
            control_msg["angular"] = 1.0
        elif keys[pygame.K_e]:
            control_msg["angular"] = -1.0

        # Serialize to binary format
        serialized_msg = struct.pack(
            "Bddd",
            control_msg["priority"],
            control_msg["x"],
            control_msg["y"],
            control_msg["angular"],
        )
        
        checksum = self.calculate_checksum(serialized_msg)
        serialized_msg_with_checksum = serialized_msg + struct.pack("B", checksum)

        if self.is_real:
            # Send over serial
            self.serial_port.write(serialized_msg_with_checksum)
        else:
            self.publish_cmd_vel(control_msg)

        # Clear the screen
        self.screen.fill((0, 0, 0))

        # Render control message status
        control_text = self.font.render(f"Control - x: {control_msg['x']}, y: {control_msg['y']}, angular: {control_msg['angular']}", True, (255, 255, 255))
        self.screen.blit(control_text, (20, 20))

        # Update the display
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = BoatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()