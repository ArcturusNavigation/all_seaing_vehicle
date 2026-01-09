#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from all_seaing_interfaces.action import Task

###

# test using:
# ros2 topic pub -1 /boat_info all_seaing_interfaces/msg/BoatInfo "{last_seen: {sec: 0, nanosec: 0}, label: 1, x_pos: 100, y_pos: 200}"

###

class ReferenceInt:
    def __init__(self, val: int):
        self.val = val

class RunTasks(Node):
    def __init__(self):
        super().__init__("run_tasks")
        self.init_tasks = [
            ActionClient(self, Task, "task_init")
        ]
        self.task_list = [
            [ActionClient(self, Task, "follow_buoy_path"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "speed_challenge"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "docking"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "mechanism_navigation"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "follow_buoy_pid"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "speed_challenge_pid"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "docking_fallback"), ReferenceInt(0), ReferenceInt(0)],
        ]
        self.term_tasks = [
            
        ]
        self.current_task = None
        self.next_task_index = -1
        self.next_init_index = ReferenceInt(0)
        self.next_term_index = ReferenceInt(0)
        self.declare_parameter("max_attempt_count", 1)
        self.max_attempt_count = self.get_parameter("max_attempt_count").get_parameter_value().integer_value

        self.pause_publisher = self.create_publisher(Bool, "pause", 10)
        self.find_task() # um idk if this is right

    def attempt_task(self, current_task, incr_on_success, incr_on_fail = None):
        self.current_task = current_task
        self.incr_on_success = incr_on_success
        self.incr_on_fail = incr_on_fail
        self.get_logger().info("Starting Task Manager States...")
        self.current_task.wait_for_server()
        self.get_logger().info(f"Starting task: {self.current_task}")
        task_goal_msg = Task.Goal()
        self.get_logger().info(f"Sending goal: {task_goal_msg}")
        send_goal_future = self.current_task.send_goal_async(
            task_goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def find_task(self):
        if self.next_init_index.val < len(self.init_tasks):
            self.attempt_task(self.init_tasks[self.next_init_index.val], self.next_init_index, None)
            return

        # print(self.task_list)

        for _ in range(len(self.task_list)):
            self.next_task_index += 1
            if self.next_task_index >= len(self.task_list):
                self.next_task_index -= len(self.task_list)
            if self.task_list[self.next_task_index][1].val == 0 and self.task_list[self.next_task_index][2].val < self.max_attempt_count:
                self.attempt_task(*self.task_list[self.next_task_index])
                return
        
        if self.next_term_index.val < len(self.term_tasks):
            self.attempt_task(self.term_tasks[self.next_term_index.val], self.next_term_index, None)
            return
        
        self.get_logger().info("All tasks completed. Shutting down node.")
        self.destroy_node()
        return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
        self.get_logger().info(f"Current task: {self.current_task}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected, looking for new task")
            if not (self.incr_on_fail is None): # consider failure to start task (goal rejected when setup conditions not met) as failure to do the task?
                self.incr_on_fail.val += 1
            self.find_task()
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Task success, marked as completed, looking for new task")
            self.incr_on_success.val += 1
        else:
            self.get_logger().info("Task failure, looking for new task")
            if not (self.incr_on_fail is None):
                self.incr_on_fail.val += 1
        self.find_task()

def main(args=None):
    rclpy.init(args=args)
    node = RunTasks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()






