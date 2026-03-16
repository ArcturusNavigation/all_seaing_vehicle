import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import FollowPath, Task, Waypoint, Search
from all_seaing_interfaces.msg import ControlOption
from all_seaing_common.action_server_base import ActionServerBase
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_controller.pid_controller import PIDController
from all_seaing_autonomy.buoy_utils import (
    InternalBuoyPair, ob_coords, get_closest_to, midpoint_pair_dir,
    split_buoys, obs_to_pos, obs_to_pos_label, filter_front_buoys,
    pick_buoy, replace_closest, buoy_pairs_to_markers, pair_to_pose
)

import os
import yaml
import time
import math
import numpy as np
from functools import partial

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

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
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

        self.declare_parameter("red_left", True)
        self.red_left = self.get_parameter("red_left").get_parameter_value().bool_value

        self.declare_parameter("gate_dist_thres", 25.0)
        self.gate_dist_thres = self.get_parameter("gate_dist_thres").get_parameter_value().double_value

        self.declare_parameter("circling_buoy_dist_thres", 25.0)
        self.circling_buoy_dist_thres = self.get_parameter("circling_buoy_dist_thres").get_parameter_value().double_value

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.circling_buoy_found = False

        self.declare_parameter("turn_offset", 5.0)
        self.turn_offset = self.get_parameter("turn_offset").get_parameter_value().double_value

        self.declare_parameter("exit_turn_eps", 0.4) #roughly a bit less than pi/6 both ways
        self.exit_turn_eps = self.get_parameter("exit_turn_eps").get_parameter_value().double_value

        self.declare_parameter("t_o_eps", 0.5)
        self.t_o_eps = self.get_parameter("t_o_eps").get_parameter_value().double_value

        self.declare_parameter("inter_buoy_pair_dist", 10.0)
        self.inter_buoy_pair_dist = self.get_parameter("inter_buoy_pair_dist").get_parameter_value().double_value

        self.declare_parameter("max_inter_gate_dist", 50.0)
        self.max_inter_gate_dist = self.get_parameter("max_inter_gate_dist").get_parameter_value().double_value

        self.max_turn_vel = (
            self.declare_parameter("max_turn_vel", [5.0, 0.0, 1.0])
            .get_parameter_value()
            .double_array_value
        )
        Turn_pid = (
            self.declare_parameter("turn_pid", [1.5, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.turn_pid = PIDController(*Turn_pid)

        self.circling_buoy_pos = np.array([0.0, 0.0])
        
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

        self.obstacles = []

        self.gate_pair = None

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
        result = self._get_result_future.result()
        if ((result is None) or
            (result.status not in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING])):
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
    
    def setup_buoys(self, pointing_direction=None, filter_front=False, visualize=True):
        """
        Runs when the first obstacle map is received, filters the buoys that are in front of
        the robot (x>0 in local coordinates) and finds (and stores) the closest green one and
        the closest red one, and because the robot is in the starting position these
        are the front buoys of the robot starting box.
        """
        self.get_logger().debug("Setting up starting buoys!")
        self.get_logger().debug(
            f"list of obstacles: {obs_to_pos_label(self.obstacles)}"
        )

        # Split all the buoys into red and green
        green_init, red_init = split_buoys(self.obstacles, self.green_labels, self.red_labels)

        # lambda function that filters the buoys that are in front of the robot
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            if  ((not filter_front) or (ob.local_point.point.x > 0)) and (np.linalg.norm(self.robot_pos - ob_coords(ob)) < self.gate_dist_thres and (pointing_direction is None or (ob_coords(ob) - self.robot_pos) @ pointing_direction > 0))
        ]
        # take the green and red buoys that are in front of the robot
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().debug(
            f"initial red buoys: {[ob_coords(buoy) for buoy in red_buoys]}, green buoys: {[ob_coords(buoy) for buoy in green_buoys]}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            self.get_logger().debug("No starting buoy pairs!")
            return False

        # From the red buoys that are in front of the robot, take the one that is closest to it.
        # And do the same for the green buoys.
        # This pair is the front pair of the starting box of the robot.
        # want to pick the pair that's far apart but has the closest midpoint
        # if self.first_setup:
        #     green_to = None
        #     red_to = None
        #     for red_b in red_buoys:
        #         for green_b in green_buoys:
        #             if self.norm(ob_coords(red_b), ob_coords(green_b)) < self.inter_buoy_pair_dist or self.norm(ob_coords(red_b), ob_coords(green_b)) > self.max_inter_gate_dist:
        #                 continue
        #             elif (green_to is None) or (self.norm(self.midpoint(ob_coords(red_b, local=True), ob_coords(green_b, local=True))) < self.norm(self.midpoint(ob_coords(red_to, local=True), ob_coords(green_to, local=True)))):
        #                 green_to = green_b
        #                 red_to = red_b
        #     if green_to is None:
        #         return False
        #     if self.ccw((0, 0), ob_coords(green_to, local=True), ob_coords(red_to, local=True)):
        #         self.red_left = True
        #         self.gate_pair = InternalBuoyPair(red_to, green_to)
        #         self.get_logger().debug("RED BUOYS LEFT, GREEN BUOYS RIGHT")
        #     else:
        #         self.red_left = False
        #         self.gate_pair = InternalBuoyPair(green_to, red_to)
        #         self.get_logger().debug("GREEN BUOYS LEFT, RED BUOYS RIGHT")
        #     self.first_setup = False
        #     return True
        # else:
        green_to = None
        red_to = None
        for red_b in red_buoys:
            for green_b in green_buoys:
                pair_dist = np.linalg.norm(ob_coords(red_b) - ob_coords(green_b))
                if pair_dist < self.inter_buoy_pair_dist or pair_dist > self.max_inter_gate_dist:
                    continue
                elif ((green_to is None) or (np.linalg.norm((ob_coords(red_b, local=True) + ob_coords(green_b, local=True)) / 2) < np.linalg.norm((ob_coords(red_to, local=True) + ob_coords(green_to, local=True)) / 2))):
                    green_to = green_b
                    red_to = red_b
        if green_to is None:
            return False
        if self.red_left:
            self.gate_pair = InternalBuoyPair(red_to, green_to)
        else:
            self.gate_pair = InternalBuoyPair(green_to, red_to)
        
        if visualize:
            self.get_logger().info(f'FOUND GATE')
            self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
            self.gate_mid, self.gate_dir = midpoint_pair_dir(self.gate_pair, 0.0)
            self.waypoint_marker_pub.publish(buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, pair_to_pose(self.gate_mid), 0.0)], self.red_left, self.global_frame_id))
        
        return True
    
    def probe_buoy(self, buoy_direction, probe_distance, buoy_detected_func):
        '''
        Function to find the buoy by moving near it (general direction).
        Keeps on appending waypoints to the north/south until it finds
        '''
        self.get_logger().info("Probing for buoy")
        max_guide_d = probe_distance
        current_guide_point = lambda: max_guide_d * buoy_direction + self.robot_pos
        self.guide_point = current_guide_point()
        self.get_logger().info(f"Current position: {self.robot_pos}. Guide point: {self.guide_point}.")

        # detection_success = self.move_to_point(self.guide_point, busy_wait=True,
        #                                         goal_update_func=partial(self.update_point, "guide_point", current_guide_point),
        #                                         exit_func=buoy_detected_func)
        detection_success = self.move_to_point(self.guide_point, busy_wait=True,
                                                exit_func=partial(buoy_detected_func, True))
        if detection_success:
            return Task.Result(success=True)
        return Task.Result(success=False)
    
    def buoy_detected(self, obstacles, buoy_labels, duplicate_dist, buoy_front=False):
        '''
        Check if the beacon for turning is detected (returns boolean).
        Also sets the position of the beacon if it is found.
        '''    
        # backup_buoy = None
        updated_pos = False
        for obstacle in obstacles:
            if obstacle.label in buoy_labels:
                if np.linalg.norm(self.robot_pos - ob_coords(obstacle)) > self.circling_buoy_dist_thres:
                    continue
                buoy_pos = ob_coords(obstacle)
                buoy_dir = buoy_pos - self.robot_pos
                dot_prod = buoy_dir @ self.robot_dir
                # if (backup_buoy is None) or (self.green_beacon_found and (self.norm(self.green_beacon_pos, buoy_pos) < self.norm(self.green_beacon_pos, backup_buoy))):
                #     backup_buoy = buoy_pos
                if ((not buoy_front) or (dot_prod > 0)) and ((not self.circling_buoy_found) or (np.linalg.norm(self.circling_buoy_pos - buoy_pos) < duplicate_dist)): #check if buoy position is behind robot i.e. dot product is negative
                    if not self.circling_buoy_found:
                        self.get_logger().info(f"Found beacon at {obstacle.global_point.point}")
                    self.circling_buoy_found = True
                    updated_pos = True
                    self.circling_buoy_pos = buoy_pos
                    robot_buoy_dist = np.linalg.norm(buoy_dir)
                    self.buoy_direction = buoy_dir / robot_buoy_dist
                    break
        # if (not updated_pos) and (backup_buoy is not None):
        #     self.get_logger().info('SWITCHING TO BACKUP GREEN BEACON BUOY')
        #     self.green_beacon_pos = backup_buoy
        return self.circling_buoy_found
    
    def circle_buoy(self, buoy_labels, adaptive_distance, duplicate_dist, buoy_loop_count=1, left_first=False):
        '''
        Function to circle the buoy.
        '''
        self.get_logger().info("Circling buoy")
        if not self.buoy_detected(self.obstacles, buoy_labels, duplicate_dist):
            self.get_logger().info("speed challenge probing exited without finding buoy")
            return Task.Result(success=False)

        # circle the buoy like a baseball diamond
        # a better way to do this might be to have the astar run to original cell,
        # but require the path to go around buoy

        t_o = self.turn_offset
        bd = np.array(self.buoy_direction)
        first_dir = np.array([bd[1], -bd[0]]) * t_o
        second_dir = bd * t_o
        third_dir = -first_dir
        fourth_dir = -second_dir

        if left_first:
            first_dir, third_dir = third_dir, first_dir

        first_base = self.circling_buoy_pos + first_dir
        second_base = self.circling_buoy_pos + second_dir
        third_base = self.circling_buoy_pos + third_dir
        fourth_base = self.circling_buoy_pos + fourth_dir

        bases = [first_base, second_base, third_base, fourth_base]
        dirs = [first_dir, second_dir, third_dir, fourth_dir]

        for i in range(buoy_loop_count):
            if i < buoy_loop_count - 1:
                loop_bases = bases  # all 4
                loop_dirs = dirs
            else:
                loop_bases = bases[:3]  # only 3 on last lap
                loop_dirs = dirs[:3]

            self.get_logger().info(f"initial moved to points= {self.moved_to_point}")
            self.get_logger().info(f"buoy pose: {self.circling_buoy_pos}")
            self.get_logger().info(f"bases: {bases}")
            self.prev_sent_buoy_pos = self.circling_buoy_pos
            for base, offset in zip(loop_bases, loop_dirs):
                # self.move_to_point(base, busy_wait=True, goal_update_func=partial(self.update_buoy_pos, self.obstacles, buoy_labels, offset, adaptive_distance))
                def update_current_point():
                    self.buoy_detected(self.obstacles, buoy_labels, duplicate_dist)
                    return self.circling_buoy_pos + offset
                self.base_point = base
                self.move_to_point(self.base_point, busy_wait=True,
                                goal_update_func=partial(self.update_point, "base_point", adaptive_distance, update_current_point) )
                self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)
    
    def smooth_circle_buoy(self, buoy_labels, adaptive_distance, duplicate_dist, target_pt, buoy_loop_count=1, left_first=False):
        '''
        Function to circle the green beacon in a smooth fashion.
        '''
        self.get_logger().info("Circling green beacon")
        if not self.buoy_detected(self.obstacles, buoy_labels, duplicate_dist):
            self.get_logger().info("speed challenge probing exited without finding green beacon")
            return Task.Result(success=False)

        t_o = self.get_parameter("turn_offset").get_parameter_value().double_value
        robot_buoy_vector = self.circling_buoy_pos - self.robot_pos
        robot_buoy_dist = np.linalg.norm(robot_buoy_vector)
        self.buoy_direction = robot_buoy_vector / robot_buoy_dist
        first_dir = np.array([self.buoy_direction[1], -self.buoy_direction[0]]) * (t_o + self.t_o_eps)
        if left_first:
            first_dir = -first_dir

        self.first_base = self.circling_buoy_pos + first_dir

        self.get_logger().info(f"initial moved to points= {self.moved_to_point}")
        self.get_logger().info(f"green beacon buoy pose: {self.circling_buoy_pos}")
        self.get_logger().info(f"first base: {self.first_base}")

        def update_first_base():
            self.buoy_detected(self.obstacles, buoy_labels, duplicate_dist)
            return self.circling_buoy_pos + first_dir
        self.move_to_point(self.first_base, busy_wait=True,
                            goal_update_func=partial(self.update_point, "first_base", adaptive_distance, update_first_base) )
        self.get_logger().info(f"moved to first base = {self.moved_to_point}")

        in_circling = False # boolean flag for whether boat is circling, set to True when boat has turned at least 90 degrees
        def exit_angle_met():
            buoy_gate_vector = target_pt - self.circling_buoy_pos
            buoy_gate_dir = buoy_gate_vector / np.linalg.norm(buoy_gate_vector)
            angle = math.atan2(self.robot_dir[1], self.robot_dir[0]) - math.atan2(buoy_gate_dir[1], buoy_gate_dir[0])
            if (angle < 0):
                angle += 2*math.pi
            return (angle < self.exit_turn_eps) or (angle > 2*math.pi-self.exit_turn_eps)

        self.turn_pid.reset()
        self.turn_pid.set_setpoint(t_o)
        self.turn_pid.set_effort_max(self.max_turn_vel[2])
        self.turn_pid.set_effort_min(-self.max_turn_vel[2])
        self.prev_update_time = self.get_clock().now()
        self.get_logger().info(f"Circling buoy via PID")

        laps_completed = 0
        in_circling = False
        was_exit_angle_met = False  # track previous state to detect transitions
        initial_robot_dir = self.robot_dir

        while laps_completed < buoy_loop_count:
            pid_output = self.turn_pid.get_effort()
            self.send_vel_cmd(self.max_turn_vel[0], 0.0, (1.0 if left_first else -1.0)*pid_output)
            self.buoy_detected(self.obstacles, buoy_labels, duplicate_dist)
            dist_to_buoy = np.linalg.norm(self.circling_buoy_pos - self.robot_pos)
            dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
            self.turn_pid.update(dist_to_buoy, dt)

             # check if robot has turned at least 90 degrees from initial heading
            if not in_circling:
                angle_from_start = math.atan2(self.robot_dir[1], self.robot_dir[0]) - math.atan2(initial_robot_dir[1], initial_robot_dir[0])
                if angle_from_start < 0:
                    angle_from_start += 2 * math.pi
                if angle_from_start > math.pi / 2 and angle_from_start < 3 * math.pi / 2:
                    in_circling = True
                    self.get_logger().info("Robot has turned 90 degrees, now tracking laps")

            currently_met = exit_angle_met()
            if in_circling and currently_met and not was_exit_angle_met:
                laps_completed += 1
                self.get_logger().info(f"Completed lap {laps_completed}/{buoy_loop_count}")
                if laps_completed < buoy_loop_count:
                    in_circling = False
                    initial_robot_dir = self.robot_dir
            was_exit_angle_met = currently_met

            time.sleep(self.timer_period)

        self.get_logger().info(f"Finished circling buoy")
        return Task.Result(success=True)

    def update_gate_wpt_pos(self, obstacles, green_labels, red_labels, duplicate_dist, forward_dist = 0.0, tryhard=False):
        # split the buoys into red and green
        green_buoys, red_buoys = split_buoys(obstacles, green_labels, red_labels)
        self.gate_pair.left, res_left_left = replace_closest(self.gate_pair.left, red_buoys if self.red_left else green_buoys, duplicate_dist)
        self.gate_pair.right, res_right_right = replace_closest(self.gate_pair.right, green_buoys if self.red_left else red_buoys, duplicate_dist)
        if tryhard:
            _, res_left_right = replace_closest(self.gate_pair.left, green_buoys if self.red_left else red_buoys, duplicate_dist)
            _, res_right_left = replace_closest(self.gate_pair.right, red_buoys if self.red_left else green_buoys, duplicate_dist)
            # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint, it is false
            if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                self.get_logger().info('WE ARE GOING TO A FAKE PAIR, FIND PATH AGAIN')
                if not self.setup_buoys():
                    return self.gate_wpt
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, forward_dist)
        self.waypoint_marker_pub.publish(buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, pair_to_pose(gate_wpt), 0.0)], self.red_left, self.global_frame_id))
        return gate_wpt