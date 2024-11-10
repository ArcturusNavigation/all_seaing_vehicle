#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from all_seaing_interfaces.action import TaskOne
import time

class Task2ActionServer(Node):
    def __init__(self):
        super().__init__('task_2')
        self._action_server = ActionServer(
            self,
            TaskOne,
            'task_2',
            self.execute_callback
        )
        
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        
        start_time = time.time()
        while time.time() - start_time < 3.0:
            # Send feedback
            feedback_msg = TaskOne.Feedback()
            feedback_msg.time_elapsed = time.time() - start_time
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.time_elapsed:.2f} seconds elapsed')
            
            # Sleep briefly before checking again
            time.sleep(0.1)
            
        # Set the result
        result = TaskOne.Result()
        result.success = True
        self.get_logger().info('Goal succeeded')
        
        goal_handle.succeed()
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = Task2ActionServer()
    rclpy.spin(action_server)

    # Shutdown
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
