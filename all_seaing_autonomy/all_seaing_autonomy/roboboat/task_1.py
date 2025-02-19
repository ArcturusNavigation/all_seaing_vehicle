#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from all_seaing_interfaces.action import Task
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
import time
import threading

from all_seaing_autonomy.roboboat.waypoint_finder import WaypointFinder

class Task1ActionServer(Node):
    def __init__(self):
        super().__init__('task_1')
        self._action_server = ActionServer(
            self,
            Task,
            'task_1',
            self.execute_callback
        )
        self.pause = False
        self.pause_cv = threading.Condition()
        self.pause_subscriber = self.create_subscription(
            Bool,
            'pause',
            self.pause_callback,
            10
        )

    def pause_callback(self, msg):
        with self.pause_cv:
            self.pause = msg.data
            if self.pause:
                self.get_logger().info('Pausing task 1')
            else:
                self.get_logger().info('Resuming task 1')
                self.pause_cv.notify_all()

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Task 1: Starting WaypointFinder node')

        # Start the WaypointFinder node in a separate thread.
        waypoint_finder = WaypointFinder()
        wf_executor = rclpy.executors.SingleThreadedExecutor()
        wf_executor.add_node(waypoint_finder)
        wf_thread = threading.Thread(target=wf_executor.spin, daemon=True)
        wf_thread.start()

        start_time = time.time()
        last_feedback_time = start_time

        # Run indefinitely, sending feedback every 5 seconds.
        while rclpy.ok():
            with self.pause_cv:
                while self.pause:
                    self.get_logger().info('Task 1 paused')
                    self.pause_cv.wait()

            current_time = time.time()

            # Send feedback every 5 seconds.
            if current_time - last_feedback_time >= 5.0:
                elapsed_time = current_time - start_time
                feedback_msg = Task.Feedback()
                feedback_msg.time_elapsed = elapsed_time
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'Feedback: {feedback_msg.time_elapsed:.2f} seconds elapsed')
                last_feedback_time = current_time

            time.sleep(1)  # Sleep briefly to avoid busy-waiting

            # Check if the goal has been canceled.
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                wf_executor.shutdown()
                waypoint_finder.destroy_node()
                goal_handle.canceled()
                result = Task.Result()
                result.success = False
                return result

        # Shutdown the WaypointFinder if the node is shutting down.
        wf_executor.shutdown()
        waypoint_finder.destroy_node()
        result = Task.Result()
        result.success = True
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = Task1ActionServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server)
    executor.spin()

    # Shutdown
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()