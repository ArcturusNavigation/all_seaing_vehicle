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



heartbeat_msg = {
    "in_teleop": True,
    "e_stopped": False,
}

def calculate_checksum(data):
    return sum(data) % 256

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                heartbeat_msg["e_stopped"] = not heartbeat_msg["e_stopped"]
            elif event.key == pygame.K_RETURN:
                heartbeat_msg["in_teleop"] = not heartbeat_msg["in_teleop"]

    
    # Serialize to binary format
    serialized_msg = struct.pack(
        "BB",
        heartbeat_msg["in_teleop"],
        heartbeat_msg["e_stopped"],
    )
    
    checksum = calculate_checksum(serialized_msg)
    serialized_msg_with_checksum = serialized_msg + struct.pack("B", checksum)

    # Send over serial
    serial_port.write(serialized_msg_with_checksum)

    # Clear the screen
    screen.fill((0, 0, 0))

    # Render heartbeat message status
    heartbeat_text = font.render(f"Heartbeat - in_teleop (enter): {heartbeat_msg['in_teleop']}, e_stopped (space): {heartbeat_msg['e_stopped']}", True, (255, 255, 255))
    screen.blit(heartbeat_text, (20, 60))

    # Update the display
    pygame.display.flip()

    pygame.time.delay(50)  # 50 ms delay

# Close serial and pygame
serial_port.close()
pygame.quit()