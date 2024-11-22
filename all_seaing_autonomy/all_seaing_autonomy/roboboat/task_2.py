#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from all_seaing_interfaces.action import Task
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
import time
import threading

class Task2ActionServer(Node):
    def __init__(self):
        super().__init__('task_2')
        self._action_server = ActionServer(
            self,
            Task,
            'task_2',
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
                self.get_logger().info('Pausing task 2')
            else:
                self.get_logger().info('Resuming task 2')
                self.pause_cv.notify_all()
    
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        
        total_time = 0.0
        while total_time < 4.0:
            with self.pause_cv:
                while self.pause:
                    self.get_logger().info('Task 2 paused')
                    self.pause_cv.wait()
            start_time = time.time()
            # Send feedback
            feedback_msg = Task.Feedback()
            feedback_msg.time_elapsed = total_time
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.time_elapsed:.2f} seconds elapsed')
            
            # Sleep briefly before checking again
            time.sleep(0.1)
            end_time = time.time()
            total_time += end_time - start_time
            
        # Set the result
        result = Task.Result()
        result.success = True
        self.get_logger().info('Goal succeeded')
        
        goal_handle.succeed()
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = Task2ActionServer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server)
    executor.spin()
    
    # Shutdown
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
