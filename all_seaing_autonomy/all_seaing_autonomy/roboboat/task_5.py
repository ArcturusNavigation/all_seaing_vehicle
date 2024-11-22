#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from all_seaing_interfaces.action import ShootBoat
import time

class Task5ActionServer(Node):
    def __init__(self):
        super().__init__('task_1')
        self._action_server = ActionServer(
            self,
            ShootBoat,
            'task_5',
            self.execute_callback
        )
        
    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        
        # Read values from the goal_handle
        goal = goal_handle.request
        self.get_logger().info(f'Received goal: {goal}')
        
        total_time = 0.0
        while total_time < 3.0:
            start_time = time.time()
            
            # Send feedback
            feedback_msg = ShootBoat.Feedback()
            feedback_msg.time_elapsed = total_time
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.time_elapsed:.2f} seconds elapsed')
            
            # Sleep briefly before checking again
            time.sleep(0.1)
            end_time = time.time()
            total_time += end_time - start_time
        
        # Set the result
        result = ShootBoat.Result()
        result.success = True
        self.get_logger().info('Goal succeeded')
        
        goal_handle.succeed()
        
        return result
    
def main(args=None):
    rclpy.init(args=args)
    action_server = Task5ActionServer()

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

    # Shutdown
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()