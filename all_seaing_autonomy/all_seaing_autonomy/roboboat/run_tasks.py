#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

# this is not really defined yet
# from all_seaing_interfaces.msg import BoatInfo

from all_seaing_interfaces.action import Task
# from all_seaing_interfaces.action import ShootBoat


###

# test using:
# ros2 topic pub -1 /boat_info all_seaing_interfaces/msg/BoatInfo "{last_seen: {sec: 0, nanosec: 0}, label: 1, x_pos: 100, y_pos: 200}"

###

class RunTasks(Node):
    def __init__(self):
        super().__init__("run_tasks")
        self.task_list = [
            # ActionClient(self, Task, "task_idle")
            # ActionClient(self, Task, "follow_buoy_path")

            ActionClient(self, Task, "task_1"),
            # ActionClient(self, Task, "task_2")
            # ActionClient(self, Task, "task_6")
        ]
        # self.task_5_signal_listener = self.create_subscription(
        #     BoatInfo,
        #     "boat_info",
        #     self.task_5_signal_callback,
        #     10
        # )

        # self.task_5_action_client = ActionClient(self, ShootBoat, "task_5")
        self.current_task = None
        self.next_task_index = 0 # by default the next task is idling

        self.pause_publisher = self.create_publisher(Bool, "pause", 10)

        self.start_task()

    # def task_5_signal_callback(self, msg):
    #     self.get_logger().info("Received signal for task 5")
    #     self.task_5_action_client.wait_for_server()

    #     task_5_goal_msg = ShootBoat.Goal()
    #     task_5_goal_msg.last_seen = msg.last_seen
    #     task_5_goal_msg.label = msg.label
    #     task_5_goal_msg.x_pos = msg.x_pos
    #     task_5_goal_msg.y_pos = msg.y_pos

    #     send_goal_future = self.task_5_action_client.send_goal_async(
    #         task_5_goal_msg,
    #         feedback_callback=self.task_5_feedback_callback
    #     )
    #     send_goal_future.add_done_callback(self.task_5_goal_response_callback)

    # def task_5_goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info("Goal rejected")
    #         return

    #     self.get_logger().info("Goal accepted")
    #     self.pause_publisher.publish(Bool(data=True))

    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.task_5_result_callback)

    # def task_5_result_callback(self, future):
    #     result = future.result().result
    #     if result.success:
    #         self.get_logger().info("Goal succeeded")
    #     else:
    #         self.get_logger().info("Goal failed")

    #     self.pause_publisher.publish(Bool(data=False))

    # def task_5_feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
    #     self.get_logger().info(f"Current task: task_5")

    def start_task(self):
        self.current_task = self.task_list[self.next_task_index]
        self.get_logger().info(f"Starting Task Manager States...")
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
            self.start_task() if self.task_list else rclpy.shutdown()
            # TODO: modify this so that when a task is finished, or the task list is empty,
            # enter a transition state to search for a new task. 
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