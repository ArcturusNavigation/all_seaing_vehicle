#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlOption
import serial
import pygame
import struct
import serial
import time

class OnshoreLoraController(Node):
    def __init__(self):
        super().__init__('onshore_lora_controller')

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("Keyboard Controller")

        self.data_size = struct.calcsize('Bddd')

        # Serial setup
        '''
        self.serial_port = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)
        time.sleep(2)  # Allow serial port to stabilize
        '''
        
        self.timer = self.create_timer(0.1, self.read_keyboard_data)

        self.declare_parameter("y", 0.0)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("angular", 0.0)

        self.y = self.get_parameter("y").value
        self.x = self.get_parameter("x").value
        self.angular = self.get_parameter("angular").value

    def read_keyboard_data(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

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
            control_msg["y"] = self.y  # Move left
        elif keys[pygame.K_d]:
            control_msg["y"] = -1.0 * self.y  # Move right
        else:
            control_msg["y"] = 0.0  # Stop horizontal movement
        
        if keys[pygame.K_w]:
            control_msg["x"] = self.x  # Move forward
        elif keys[pygame.K_s]:
            control_msg["x"] = -1.0 * self.x  # Move backward
        else:
            control_msg["x"] = 0.0  # Stop vertical movement
            
        if keys[pygame.K_q]:
            control_msg["angular"] = self.angular
        elif keys[pygame.K_e]:
            control_msg["angular"] = -1.0 * self.angular
        else:
            control_msg["angular"] = 0.0 

        # Serialize to binary format
        serialized_msg = struct.pack(
            "Bddd",
            control_msg["priority"],
            control_msg["x"],
            control_msg["y"],
            control_msg["angular"],
        )

        # Send over serial
        self.serial_port.write(serialized_msg)
        
        # Print sent message for debugging
        print(f"Sent: {control_msg}")

        pygame.time.delay(100)  # 100 ms delay

def main(args=None):
    rclpy.init(args=args)
    serial_control_subscriber = OnshoreLoraController()

    try:
        rclpy.spin(serial_control_subscriber)
    except KeyboardInterrupt:
        pass

    # Close serial connection
    serial_control_subscriber.serial_port.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    
    # Adjust control message based on arrow keys
    if keys[pygame.K_a]:
        control_msg["y"] = 1.0  # Move left
    elif keys[pygame.K_d]:
        control_msg["y"] = -1.0  # Move right
    else:
        control_msg["y"] = 0.0  # Stop horizontal movement
    
    if keys[pygame.K_w]:
        control_msg["x"] = 1.0  # Move forward
    elif keys[pygame.K_s]:
        control_msg["x"] = -1.0  # Move backward
    else:
        control_msg["x"] = 0.0  # Stop vertical movement
        
    if keys[pygame.K_q]:
        control_msg["angular"] = 1.0
    elif keys[pygame.K_e]:
        control_msg["angular"] = -1.0
    else:
        control_msg["angular"] = 0.0

    # Serialize to binary format
    serialized_msg = struct.pack(
        "Bddd",
        control_msg["priority"],
        control_msg["x"],
        control_msg["y"],
        control_msg["angular"],
    )

    # Send over serial
    serial_port.write(serialized_msg)
    
    # Print sent message for debugging
    print(f"Sent: {control_msg}")

    pygame.time.delay(100)  # 100 ms delay

# Close serial and pygame
serial_port.close()
pygame.quit()
'''