#!/usr/bin/env python3

import pygame
import struct
import serial
import time

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 400))
pygame.display.set_caption("Keyboard Controller")
font = pygame.font.Font(None, 36)

# Serial setup
serial_port = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)
time.sleep(2)  # Allow serial port to stabilize

def calculate_checksum(data):
    return sum(data) % 256

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

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
        control_msg["x"] = 1.0  # Move forward
    elif keys[pygame.K_s]:
        control_msg["x"] = -1.0  # Move backward
        
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
    
    checksum = calculate_checksum(serialized_msg)
    serialized_msg_with_checksum = serialized_msg + struct.pack("B", checksum)

    # Send over serial
    serial_port.write(serialized_msg_with_checksum)

    # Clear the screen
    screen.fill((0, 0, 0))

    # Render control message status
    control_text = font.render(f"Control - x: {control_msg['x']}, y: {control_msg['y']}, angular: {control_msg['angular']}", True, (255, 255, 255))
    screen.blit(control_text, (20, 20))

    # Update the display
    pygame.display.flip()

    pygame.time.delay(50)  # 50 ms delay

# Close serial and pygame
serial_port.close()
pygame.quit()