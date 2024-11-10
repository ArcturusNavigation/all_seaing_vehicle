#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from all_seaing_interfaces.action import TaskOne


# class RunTasks(Node):
#     def __init__(self):
#         super().__init__('run_tasks')
#         self.task_one = ActionClient(self, TaskOne, 'task_1')
#         self.task_two = ActionClient(self, TaskOne, 'task_2')
        

#     def start_task_1(self):
#         self.get_logger().info("Starting sequential actions...")
        
#         self.task_one.wait_for_server()
#         task_1_goal_msg = TaskOne.Goal()   
        
#         send_goal_future = self.task_one.send_goal_async(
#             task_1_goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         send_goal_future.add_done_callback(self.goal_response_callback_1)  
        
           
        
#     def goal_response_callback_1(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             return

#         self.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback_1)

#     def get_result_callback_1(self, future):
#         result = future.result().result
#         if result.success:
#             self.get_logger().info('Goal succeeded!')
#             self.start_task_2()
#         else:
#             self.get_logger().info('Goal failed')
        
        
#     def start_task_2(self):
#         self.get_logger().info("Starting sequential actions...")
        
#         self.task_two.wait_for_server()
#         task_2_goal_msg = TaskOne.Goal()   
        
#         send_goal_future = self.task_two.send_goal_async(
#             task_2_goal_msg,
#             feedback_callback=self.feedback_callback
#         )
#         send_goal_future.add_done_callback(self.goal_response_callback_2)  
        
#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.get_logger().info(f'Received feedback: {feedback.time_elapsed:.2f} seconds elapsed')
        
#     def goal_response_callback_2(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             return

#         self.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback_2)

#     def get_result_callback_2(self, future):
#         result = future.result().result
#         if result.success:
#             self.get_logger().info('Goal succeeded!')
#         else:
#             self.get_logger().info('Goal failed')
        
#         # Shut down after receiving the result
#         rclpy.shutdown()
        
class RunTasks(Node):
    def __init__(self):
        super().__init__('run_tasks')
        self.task_list = [
            ActionClient(self, TaskOne, 'task_1'),
            ActionClient(self, TaskOne, 'task_2')
        ]
        
    def start_task(self):
        task = self.task_list.pop(0)
        self.get_logger().info("Starting sequential actions...")
        task.wait_for_server()
        task_goal_msg = TaskOne.Goal()
        send_goal_future = task.send_goal_async(
            task_goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.time_elapsed:.2f} seconds elapsed')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
            self.start_task() if self.task_list else rclpy.shutdown()
        else:
            self.get_logger().info('Goal failed')

       
def main(args=None):
    
    
    rclpy.init(args=args)
    node = RunTasks()

    try:
        node.start_task()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()