#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import KeyboardButton 

import time


TIMER_PERIOD = 1 / 30

class RunTasks(Node):
    def __init__(self):
        super().__init__("run_tasks")
        self.task_list = [
            ActionClient(self, Task, "follow_buoy_pid"),
        ]

        self.goal_handle = None
        self.current_task = None
        self.next_task_index = 0

        self.keyboard_sub = self.create_subscription(
            KeyboardButton, "/keyboard_button", self.real_keyboard_callback, 10
        )

        self.p_pressed = False
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def real_keyboard_callback(self, msg):
        if msg.key == "p":
            if self.goal_handle is not None:
                future = self.goal_handle.cancel_goal_async()
                future.add_done_callback(self.reset_state)
            else:
                self.p_pressed = True

    def reset_state(self, _):
        self.next_task_index = 0
        self.p_pressed = True

    def timer_callback(self):
        if not self.p_pressed:
            return

        self.p_pressed = False
        self.current_task = self.task_list[self.next_task_index]
        self.get_logger().info("Starting Task Manager State...")
        self.current_task.wait_for_server()
        self.get_logger().info(f"Starting task: {self.current_task._action_name}")
        send_goal_future = self.current_task.send_goal_async(
            Task.Goal(),
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
        self.get_logger().info(f"Current task: {self.current_task._action_name}")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal succeeded!")
            self.next_task_index += 1
            self.next_task_index %= len(self.task_list)
        else:
            self.get_logger().info("Goal failed")


def main(args=None):
    rclpy.init(args=args)
    node = RunTasks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
