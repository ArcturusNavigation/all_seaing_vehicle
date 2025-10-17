#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse 
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import Task
from all_seaing_common.action_server_base import ActionServerBase

import os
import yaml
import time
import math

class TaskServerBase(ActionServerBase):
    def __init__(self, server_name, timer_period = 1.0 / 30.0):
        super().__init__(server_name)
        self.server_name = server_name

        self._action_server = ActionServer(
            self,
            Task,
            server_name,
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
            goal_callback=self.goal_callback
        )

        self.timer_period = timer_period

    # Mark result of task as successful, exit control loop
    # IN THEORY, supports directly marking .result as true, and will mark _succeed as true by default
    def mark_successful(self):
        self._succeed = True
        self.result = True
    
    # Mark result of task as unsuccessful (useful if we start a task and realize something went wrong), exit control loop
    def mark_unsuccessful(self):
        self._succeed = False
        self.result = True

    # Return True if we should accept the request in goal_request
    def should_accept_task(self, goal_request):
        return True

    # Initial setup - implement here, do initial setup
    def init_setup(self):
        self.mark_successful()

    # Control loop - implement here
    def control_loop(self):
        pass

# Functions below are not meant to be reimplemented

    def goal_callback(self, goal_request):
        self.get_logger().info('Task Server [{self.server_name}] received task')
        if self.should_accept_task(goal_request):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback(self, goal_handle):
        self.start_process(f"Task Server [{self.server_name}] started task with goal handle {goal_handle}")

        self.result = False
        self._succeed = True

        while (not self.result) and rclpy.ok():
            time.sleep(self.timer_period)
            if self.should_abort():
                self.end_process(f"Task Server [{self.server_name}] aborted due to new request in setup")
                goal_handle.abort()
                return Task.Result()
            if goal_handle.is_cancel_requested:
                self.end_process(f"Task Server [{self.server_name}] cancelled due to request cancellation in setup")
                goal_handle.canceled()
                return Task.Result()

        if not self.result:
            self.get_logger().info("ROS shutdown detected or loop ended unexpectedly in setup")
            goal_handle.abort()
            return Task.Result(success=False)

        if (not self._succeed):
            self.end_process(f"Task Server [{self.server_name}] task failed in setup")
            goal_handle.abort()
            return Task.Result(success=False)

        self.result = False
        self._succeed = True

        while (not self.result) and rclpy.ok():

            if self.should_abort():
                self.end_process(f"Task Server [{self.server_name}] aborted due to new request in control")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process(f"Task Server [{self.server_name}] cancelled due to request cancellation in control")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)
        
        if not self.result:
            self.get_logger().info("ROS shutdown detected or loop ended unexpectedly in control.")
            goal_handle.abort()
            return Task.Result(success=False)

        self.end_process(f"Task Server [{self.server_name}] task completed with result {self._succeed}")
        goal_handle.succeed()
        return Task.Result(success=self._succeed)