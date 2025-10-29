#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import KeyboardButton 
from task_server_base import TaskServerBase

from sensor_msgs.msg import Joy
import time

class TaskInitServer(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "task_init_server", action_name = "task_init", timer_period = 1.0)

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.keyboard_sub = None
        if self.is_sim: 
            self.get_logger().info("Running in simulation mode. Listening to joystick input.")
            self.keyboard_sub = self.create_subscription(
                Joy, "/joy", self.sim_keyboard_callback, 10
            )
        else: 
            self.get_logger().info("Running in real mode. Listening to keyboard input.")
            self.keyboard_sub = self.create_subscription(
                KeyboardButton, "/keyboard_button", self.real_keyboard_callback, 10
            )
        self.p_pressed = False
    
    def control_loop(self):
        if self.p_pressed:
            self.get_logger().info("You pressed 'p'. Moving onto next task")
            self.p_pressed = False
            self.mark_successful()
    
    def sim_keyboard_callback(self, msg):
        # self.get_logger().info("sim keyboard cb")
        if msg.buttons[2]: 
            # self.get_logger().info("hihihihi")
            self.p_pressed = True
    
    def real_keyboard_callback(self, msg):
        if msg.key == "p":
            self.p_pressed = True

def main(args=None):
    rclpy.init(args=args)
    node = TaskInitServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()