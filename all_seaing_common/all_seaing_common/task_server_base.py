import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import FollowPath, Task, Waypoint, Search
from all_seaing_interfaces.msg import ControlOption
from all_seaing_common.action_server_base import ActionServerBase
from action_msgs.msg import GoalStatus

from all_seaing_common.report_pb2 import LatLng
import all_seaing_common.report_pb2

import os
import yaml
import time
import math

class TaskServerBase(ActionServerBase):
    def __init__(self, server_name, action_name, search_action_name):
        super().__init__(server_name)
        self.server_name = server_name

        self._action_server = ActionServer(
            self,
            Task,
            action_name,
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
            goal_callback=self.goal_callback
        )

        self._search_server = ActionServer(
            self,
            Search,
            search_action_name,
            execute_callback=self.search_execute_callback,
            cancel_callback=self.default_cancel_callback,
            goal_callback=self.search_goal_callback
        )

        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")
        self.control_pub = self.create_publisher(
            ControlOption, 
            "control_options", 
            10
        )

        self.timer_period = (
            self.declare_parameter("timer_period", 1.0 / 30.0)
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("xy_threshold", 1.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("wpt_theta_threshold", 10.0)
        self.declare_parameter("goal_tol", 1.0)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 10)
        self.declare_parameter("planner", "astar")

        self.declare_parameter("bypass_planner", False)

        self.bypass_planner = self.get_parameter("bypass_planner").get_parameter_value().bool_value

        self.declare_parameter("search_task_radius", 50.0)
        self.search_task_radius = self.get_parameter("search_task_radius").get_parameter_value().double_value
        
        self.moved_to_point = False
        self.waypoint_rejected = False
        self.waypoint_aborted = False
        self.sent_waypoint = None
        self.active_future_request = 0 # active future requests
        self.active_waypoint_request = 0 # keeps track of the number of active waypoint requests

        self._get_result_future = None
        self.wpt_goal_handle = None

        self.first_run = True
        self.paused = True

    # Mark result of task as successful, exit control loop
    # IN THEORY, supports directly marking .result as true, and will mark _succeed as true by default
    def mark_successful(self):
        self._succeed = True
        self.result = True
    
    # Mark result of task as unsuccessful (useful if we start a task and realize something went wrong), exit control loop or setup loop
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
        self.get_logger().info(f'Task Server [{self.server_name}] received task')
        if self.should_accept_task(goal_request):
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def execute_callback(self, goal_handle):
        self.start_process(f"Task Server [{self.server_name}] started task with goal handle {goal_handle}")

        self.goal_handle = goal_handle

        self.paused = False

        self.result = False
        self._succeed = True

        firstLoop = True

        while (not self.result) and rclpy.ok():
            if not firstLoop:
                time.sleep(self.timer_period)
            firstLoop = False
            if self.should_abort():
                self.end_process(f"Task Server [{self.server_name}] aborted due to new request in setup")
                self.paused = True
                goal_handle.abort()
                return Task.Result()
            if goal_handle.is_cancel_requested:
                self.end_process(f"Task Server [{self.server_name}] cancelled due to request cancellation in setup")
                self.paused = True
                goal_handle.canceled()
                return Task.Result()
            
            self.init_setup()

        if not self.result:
            self.get_logger().info("ROS shutdown detected or loop ended unexpectedly in setup")
            self.paused = True
            goal_handle.abort()
            return Task.Result(success=False)

        if (not self._succeed):
            self.end_process(f"Task Server [{self.server_name}] task failed in setup")
            self.paused = True
            goal_handle.abort()
            return Task.Result(success=False)

        self.result = False
        self._succeed = True

        while (not self.result) and rclpy.ok():

            if self.should_abort():
                self.end_process(f"Task Server [{self.server_name}] aborted due to new request in control")
                self.paused = True
                self.cancel_navigation()
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process(f"Task Server [{self.server_name}] cancelled due to request cancellation in control")
                goal_handle.canceled()
                self.cancel_navigation()
                self.paused = True
                self.first_run = False
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)

        if goal_handle.is_cancel_requested:
            self.end_process(f"Task Server [{self.server_name}] cancelled due to request cancellation in control")
            goal_handle.canceled()
            self.cancel_navigation()
            self.paused = True
            self.first_run = False
            return Task.Result()
        
        if not self.result:
            self.get_logger().info("ROS shutdown detected or loop ended unexpectedly in control.")
            goal_handle.abort()
            self.paused = True
            return Task.Result(success=False)

        self.end_process(f"Task Server [{self.server_name}] task completed with result {self._succeed}")
        goal_handle.succeed()
        self.paused = True
        return Task.Result(success=self._succeed)
    
    def _send_goal(self, goal_msg):
        self.follow_path_client.wait_for_server()
        self.active_future_request += 1
        self.send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg
        )
        self.send_goal_future.add_done_callback(self.follow_path_response_cb)

    def _send_wpt_goal(self, goal_msg):
        self.waypoint_client.wait_for_server()
        self.active_future_request += 1
        self.send_goal_future = self.waypoint_client.send_goal_async(
            goal_msg
        )
        self.send_goal_future.add_done_callback(self.follow_path_response_cb)

    def move_to_point(self, point, is_stationary=False, busy_wait=False, exit_func=None, goal_update_func=None, cancel_on_exit=False):
        '''
        Moves the boat to the specified position using the follow path action server.
        If busy_wait=true, then the returned truth value indicates success of point following

        Busy waits until the boat moved to the point (bad, should be fixed with asyncio patterns)

        Returns true if exit condition is met (exit_func)
        Sends new waypoint if desired by the goal_update_func
        - goal_update_func() -> should_update, (new_goal.x, new_goal.y)
        '''
        self.get_logger().info(f"Moving to point {point}")
        self.moved_to_point = False
        self.waypoint_rejected = False
        self.waypoint_aborted = False
        self.follow_path_client.wait_for_server()
        goal_msg = FollowPath.Goal()
        goal_msg.planner = self.get_parameter("planner").value
        goal_msg.x = point[0]
        goal_msg.y = point[1]
        goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
        goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
        goal_msg.goal_tol = self.get_parameter("goal_tol").value
        goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
        goal_msg.choose_every = self.get_parameter("choose_every").value
        goal_msg.is_stationary = is_stationary

        self._send_goal(goal_msg)
        if busy_wait:
            while not self.moved_to_point:
                if (exit_func is not None) and exit_func():
                    if cancel_on_exit:
                        self.cancel_navigation()
                    return True
                if (goal_update_func is not None):
                    update_goal, new_goal = goal_update_func()
                    if update_goal:
                        goal_msg.x = new_goal[0]
                        goal_msg.y = new_goal[1]
                        self.get_logger().info('ADAPTING GOAL POINT')
                        self._send_goal(goal_msg)
                if self.waypoint_rejected or self.waypoint_aborted:  # Retry functionality
                    self.get_logger().info('RESENDING GOAL')
                    self._send_goal(goal_msg)
                    self.waypoint_rejected = False
                    self.waypoint_aborted = False
                if self.goal_handle.is_cancel_requested:
                    self.cancel_navigation()
                    return False
                time.sleep(self.timer_period)
        return False

    def follow_path_response_cb(self, future):
        '''
        Responds to follow path action server goal response.
        '''
        self.wpt_goal_handle = future.result()
        self.active_future_request -= 1
        if self.active_future_request > 0:
            self.get_logger().info('PREVIOUS FUTURE, IGNORING')
            return
        
        if not self.wpt_goal_handle.accepted:
            self.get_logger().info('Waypoint rejected')
            self.waypoint_rejected = True
            return

        self.get_logger().info("Waypoint accepted")
        self._get_result_future = self.wpt_goal_handle.get_result_async()
        self.active_waypoint_request += 1
        self._get_result_future.add_done_callback(self.get_point_result_cb)

    def cancel_navigation(self):
        self.get_logger().info('TRYING TO CANCEL NAVIGATION')
        if self._get_result_future is None or self.wpt_goal_handle is None: # TODO how to handle waypoint sent (& goal handle received) after us trying to cancel it? maybe ROS delay + retry for 3 times or smth
            self.get_logger().info('FUTURE IS NONE')
            return
        if ((self._get_result_future.result() is None) or 
            (self._get_result_future.result().status not in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING])):
            self.wpt_goal_handle.cancel_goal_async()
            self.get_logger().info('CANCELLED NAVIGATION')

    def get_point_result_cb(self, future):
        '''
        Flags the path following as complete for move_to_point
        '''
        self.active_waypoint_request -= 1
        if self.active_waypoint_request > 0:
            self.get_logger().info('PREVIOUS WAYPOINT, IGNORING')
            return
        
        # Marks path following as finished/ moved to path following point
        # if path following is interrupted, does not affect moved to point
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info(f'WAYPOINT ABORTED')
            self.waypoint_aborted = True
        elif result.is_finished:
            self.get_logger().info(f'MOVED TO WAYPOINT')
            self.moved_to_point = True

    def move_to_waypoint(self, point, is_stationary=False, busy_wait=False, exit_func=None, goal_update_func=None, cancel_on_exit=False):
        self.get_logger().info(f"Moving to waypoint {point}")
        self.moved_to_point = False
        goal_msg = Waypoint.Goal()
        goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
        goal_msg.theta_threshold = self.get_parameter("wpt_theta_threshold").value
        goal_msg.x = point[0]
        goal_msg.y = point[1]
        if len(point) >= 3:
            goal_msg.theta = point[2]
            goal_msg.ignore_theta = False
        else:
            goal_msg.ignore_theta = True
        goal_msg.is_stationary = is_stationary
        self._send_wpt_goal(goal_msg)
        if busy_wait:
            while not self.moved_to_point:
                if (exit_func is not None) and exit_func():
                    if cancel_on_exit:
                        self.cancel_navigation()
                    return True
                if (goal_update_func is not None):
                    update_goal, new_goal = goal_update_func()
                    if update_goal:
                        goal_msg.x = new_goal[0]
                        goal_msg.y = new_goal[1]

                        self.get_logger().info('ADAPTING WAYPOINT')
                        self._send_wpt_goal(goal_msg)
                if self.goal_handle.is_cancel_requested:
                    self.cancel_navigation()
                    return False
                time.sleep(self.timer_period)
        return False

    def send_waypoint_to_server(self, waypoint, is_stationary=False):
        self.sent_waypoint = waypoint

        if not self.bypass_planner:
            self.move_to_point(waypoint, is_stationary=is_stationary)
        else:
            self.move_to_waypoint(waypoint, is_stationary=is_stationary)
    
    def send_vel_cmd(self, x=0, y=0, theta=0):
        '''
        Send velocity commands to be executed by the controller.
        '''
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = x
        control_msg.twist.linear.y = y
        control_msg.twist.angular.z = theta
        self.control_pub.publish(control_msg)

    def update_point(self, point_name, adaptive_distance, update_func):
        '''
        update_func: returns the new point value (x,y)
        point_name: the attribute name of the point
        Provides a wrapper for point updaters to be passed into move_to_point
        '''
        new_point = update_func()
        if not hasattr(self, point_name):
            setattr(self, point_name, new_point)
            return False, None
        else:
            # already exists attribute
            old_point = getattr(self, point_name)
            dist_squared = (old_point[0] - new_point[0])**2 + (old_point[1] - new_point[1])**2
            if (math.sqrt(dist_squared) > adaptive_distance):
                setattr(self, point_name, new_point)
                return True, new_point
            return False, None
        
    def search_goal_callback(self, goal_request):
        self.get_logger().info(f'Searching Server for [{self.server_name}] called')
        return GoalResponse.ACCEPT

    def search_execute_callback(self, goal_handle):
        self.start_process(f"Searching Server for [{self.server_name}] started with goal handle {goal_handle}")
        
        self.found_task = False
        
        if math.sqrt((self.robot_pos[0]-goal_handle.request.x)**2 + (self.robot_pos[1]-goal_handle.request.y)**2) < self.search_task_radius and self.should_accept_task(None):
            self.found_task = True

        if not self.found_task:
            self.get_logger().info(f"Moving to point {(goal_handle.request.x, goal_handle.request.y)}")
            self.moved_to_point = False
            self.waypoint_rejected = False
            self.waypoint_aborted = False
            self.follow_path_client.wait_for_server()
            goal_msg = FollowPath.Goal()
            goal_msg.planner = self.get_parameter("planner").value
            goal_msg.x = goal_handle.request.x
            goal_msg.y = goal_handle.request.y
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
            goal_msg.goal_tol = self.get_parameter("goal_tol").value
            goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
            goal_msg.choose_every = self.get_parameter("choose_every").value
            goal_msg.is_stationary = False

            self._send_goal(goal_msg)

        while (not self.found_task) and (not self.moved_to_point) and rclpy.ok():
            if self.should_abort():
                self.end_process(f"Searching Server for [{self.server_name}] aborted due to new request in control")
                self.cancel_navigation()
                goal_handle.abort()
                return Search.Result()

            if goal_handle.is_cancel_requested:
                self.end_process(f"Searching Server for [{self.server_name}] cancelled due to request cancellation in control")
                self.cancel_navigation()
                goal_handle.canceled()
                return Search.Result()

            if math.sqrt((self.robot_pos[0]-goal_handle.request.x)**2 + (self.robot_pos[1]-goal_handle.request.y)**2) < self.search_task_radius and self.should_accept_task(None):
                self.found_task = True
                self.cancel_navigation()
            elif self.waypoint_rejected or self.waypoint_aborted:  # Retry functionality
                self.get_logger().info('RESENDING GOAL')
                self._send_goal(goal_msg)
                self.waypoint_rejected = False
                self.waypoint_aborted = False
            time.sleep(self.timer_period)

        if (not self.found_task) and goal_handle.request.include_theta:
            self.get_logger().info(f"Moving to waypoint {(goal_handle.request.x, goal_handle.request.y, goal_handle.request.theta)}")
            self.moved_to_point = False
            goal_msg = Waypoint.Goal()
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("wpt_theta_threshold").value
            goal_msg.x = goal_handle.request.x
            goal_msg.y = goal_handle.request.y
            goal_msg.theta = goal_handle.request.theta
            goal_msg.ignore_theta = False
            goal_msg.is_stationary = False
            self._send_wpt_goal(goal_msg)

            while (not self.found_task) and (not self.moved_to_point) and rclpy.ok():
                if self.should_abort():
                    self.end_process(f"Searching Server for [{self.server_name}] aborted due to new request in control")
                    self.cancel_navigation()
                    goal_handle.abort()
                    return Search.Result()

                if goal_handle.is_cancel_requested:
                    self.end_process(f"Searching Server for [{self.server_name}] cancelled due to request cancellation in control")
                    self.cancel_navigation()
                    goal_handle.canceled()
                    return Search.Result()

                if self.should_accept_task(None):
                    self.found_task = True
                    self.cancel_navigation()
                time.sleep(self.timer_period)
        
        if not self.found_task:
            wait_start_time = time.time()
            self.get_logger().info(f"Waiting...")
        while (not self.found_task) and (time.time() - wait_start_time < goal_handle.request.wait_time) and rclpy.ok():
            if self.should_abort():
                self.end_process(f"Searching Server for [{self.server_name}] aborted due to new request in control")
                self.cancel_navigation()
                goal_handle.abort()
                return Search.Result()

            if goal_handle.is_cancel_requested:
                self.end_process(f"Searching Server for [{self.server_name}] cancelled due to request cancellation in control")
                self.cancel_navigation()
                goal_handle.canceled()
                return Search.Result()

            if self.should_accept_task(None):
                self.found_task = True
                self.cancel_navigation()
            time.sleep(self.timer_period)
        
        if goal_handle.is_cancel_requested:
            self.end_process(f"Searching Server for [{self.server_name}] cancelled due to request cancellation in control")
            self.cancel_navigation()
            goal_handle.canceled()
            return Search.Result()

        self.end_process(f"Searching Server for [{self.server_name}] task completed with result {self.found_task}")
        goal_handle.succeed()
        return Search.Result(success=self.found_task)
    
