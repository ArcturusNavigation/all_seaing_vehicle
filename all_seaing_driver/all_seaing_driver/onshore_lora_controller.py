#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
import struct
import serial
import time

class OnshoreLoraController(Node):
    def __init__(self):
        super().__init__('onshore_lora_controller')

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 400))
        pygame.display.set_caption("Keyboard Controller")
        self.font = pygame.font.Font(None, 36)

        # Serial setup
        self.serial_port = serial.Serial("/dev/ttyUSB0", 57600, timeout=1)
        time.sleep(0.5)

        # Initialize messages
        self.keyboard_msg = {
            "key": ord("0"),
        }
        self.heartbeat_msg = {
            "in_teleop": True,
            "e_stopped": False,
        }

        # Initialize Timer to Read Keyboard Inputs
        self.timer = self.create_timer(0.05, self.read_keyboard_data)  # 50ms like the original

        # ROS2 Parameters
        self.declare_parameter("y", 0.8)
        self.declare_parameter("x", 1.0)
        self.declare_parameter("angular", 0.3)
        self.y = self.get_parameter("y").value
        self.x = self.get_parameter("x").value
        self.angular = self.get_parameter("angular").value

    def read_keyboard_data(self):
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.heartbeat_msg["e_stopped"] = not self.heartbeat_msg["e_stopped"]
                elif event.key == pygame.K_RETURN:
                    self.heartbeat_msg["in_teleop"] = not self.heartbeat_msg["in_teleop"]

                if event.key == pygame.K_p:
                    self.keyboard_msg["key"] = ord("p")
                else:
                    self.keyboard_msg["key"] = ord("0")

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
            control_msg["y"] = -self.y  # Move right
        else:
            control_msg["y"] = 0.0

        if keys[pygame.K_w]:
            control_msg["x"] = self.x  # Move forward
        elif keys[pygame.K_s]:
            control_msg["x"] = -self.x  # Move backward
        else:
            control_msg["x"] = 0.0

        if keys[pygame.K_q]:
            control_msg["angular"] = self.angular
        elif keys[pygame.K_e]:
            control_msg["angular"] = -self.angular
        else:
            control_msg["angular"] = 0.0

        # Serialize to binary format (matching original: BdddBB)
        serialized_msg = struct.pack(
            "BdddBBB",
            control_msg["priority"],
            control_msg["x"],
            control_msg["y"],
            control_msg["angular"],
            self.heartbeat_msg["in_teleop"],
            self.heartbeat_msg["e_stopped"],
            self.keyboard_msg["key"],
        )

        checksum = self.calculate_checksum(serialized_msg)
        serialized_msg_with_checksum = serialized_msg + struct.pack("B", checksum)

        # Send over serial
        self.serial_port.write(serialized_msg_with_checksum)

        # Clear the screen
        self.screen.fill((0, 0, 0))

        # Render control message status
        control_text = self.font.render(f"Control - x: {control_msg['x']}, y: {control_msg['y']}, angular: {control_msg['angular']}", True, (255, 255, 255))
        self.screen.blit(control_text, (20, 20))

        # Render heartbeat message status
        heartbeat_text = self.font.render(f"Heartbeat - in_teleop (Enter): {self.heartbeat_msg['in_teleop']}, e_stopped (Space): {self.heartbeat_msg['e_stopped']}", True, (255, 255, 255))
        self.screen.blit(heartbeat_text, (20, 60))

        # Update the display
        pygame.display.flip()

    def close(self):
        """Close resources"""
        self.serial_port.close()
        pygame.quit()

    def calculate_checksum(self, data):
        """Calculate checksum for data integrity"""
        return sum(data) % 256


def main(args=None):
    rclpy.init(args=args)
    node = OnshoreLoraController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
