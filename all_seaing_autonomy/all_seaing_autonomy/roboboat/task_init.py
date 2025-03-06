#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import KeyboardButton 
from all_seaing_common.action_server_base import ActionServerBase
from sensor_msgs.msg import Joy
import time

class TaskInitServer(ActionServerBase):
    def __init__(self):
        super().__init__("task_init_server")
        self.task_init_server = ActionServer(
            self,
            Task,
            "task_init",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

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
        # self.timer_period = 1 / 10
        self.timer_period = 1.0
    
    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing task initialization...")
        goal = goal_handle.request
        self.get_logger().info(f"Received goal: {goal}")
        
        while rclpy.ok():
            self.get_logger().info("looping!!!!!!!")
            # Check if the action client requested cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested. Aborting task initialization.")
                goal_handle.canceled()
                return Task.Result(success=False)

            # Check for 'p' key press (non-blocking)
            if self.p_pressed:
                self.get_logger().info("You pressed 'p'. Moving onto next task")
                goal_handle.succeed()
                result = Task.Result()
                result.success = True
                self.p_pressed = False 
                return result

            time.sleep(self.timer_period)

        # If we exit the `while rclpy.ok()` loop somehow
        self.get_logger().info("ROS shutdown detected or loop ended unexpectedly.")
        goal_handle.abort()
        return Task.Result(success=False)

    
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