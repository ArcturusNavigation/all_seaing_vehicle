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

class RunTasks(Node):
    def __init__(self):
        super().__init__("run_tasks")
        self.task_list = [
            ActionClient(self, Task, "task_init"),
            ActionClient(self, Task, "follow_buoy_path"),
            ActionClient(self, Task, "docking_task"),
        ]
        self.current_task = None
        self.idle_index = 0
        self.end_index = -1
        self.next_task_index = self.idle_index # by default the next task is idling

        self.pause_publisher = self.create_publisher(Bool, "pause", 10)
        self.start_task() # um idk if this is right

    def start_task(self):
        self.current_task = self.task_list[self.next_task_index]
        self.get_logger().info("Starting Task Manager States...")
        self.current_task.wait_for_server()
        self.get_logger().info(f"Starting task: {self.current_task._action_name}")
        task_goal_msg = Task.Goal()
        self.get_logger().info(f"Sending goal: {task_goal_msg}")
        send_goal_future = self.current_task.send_goal_async(
            task_goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
        self.get_logger().info(f"Current task: {self.current_task._action_name}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Goal succeeded!")
            
            # TODO: modify this (self.end_index + len(self.task_list)) % len(self.task_list)so that when a task is finished, or the task list is empty,
            # enter a transition state to search for a new task. 

            # if self.next_task_index != self.idle_index:
            #     self.next_task_index = self.idle_index
            # else:
                # self.next_task_index = result.next_task_index 
                # to be implemented in idle action server?
            self.next_task_index += 1

            if self.next_task_index == len(self.task_list): # TODO: modify this to use end_index in the future.  
                # when end, shut down the node
                self.get_logger().info("All tasks completed. Shutting down node.")
                self.destroy_node()
                # should we also shutdown ros here?
                return 
            self.start_task()
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