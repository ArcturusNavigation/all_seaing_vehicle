#!/usr/bin/env python3
from ast import Num
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


from all_seaing_interfaces.msg import ObstacleMap, Obstacle, LabeledBoundingBox2DArray, LabeledBoundingBox2D
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion

import os
import yaml
import math
import time
from collections import deque
from functools import partial

class InternalBuoyPair:
    def __init__(self, left_buoy=None, right_buoy=None):
        if left_buoy is None:
            self.left = Obstacle()
        else:
            self.left = left_buoy

        if right_buoy is None:
            self.right = Obstacle()
        else:
            self.right = right_buoy

class SpeedChallenge(ActionServerBase):
    def __init__(self):
        super().__init__("speed_challenge_server")

        self._action_server = ActionServer(
            self,
            Task,
            "speed_challenge",
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.default_cancel_callback,
        )

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/labeled", self.map_cb, 10
        )


        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("xy_threshold", 1.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 10)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")
        self.declare_parameter("probe_distance", 10)
        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("inter_buoy_pair_dist", 1.0)
        self.inter_buoy_pair_dist = self.get_parameter("inter_buoy_pair_dist").get_parameter_value().double_value

        self.declare_parameter("init_gate_dist", 1.0)
        self.init_gate_dist = self.get_parameter("init_gate_dist").get_parameter_value().double_value

        self.declare_parameter("gate_dist_back", 1.0)
        self.forward_dist_back = self.get_parameter("gate_dist_back").get_parameter_value().double_value

        self.declare_parameter("timer_period", 1/60.0)
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        self.declare_parameter("is_sim", False)
        self.declare_parameter("turn_offset", 5.0)

        self.home_pos = (0, 0)
        self.blue_buoy_pos = (0, 0)
        self.runnerActivated = False
        
        self.buoy_direction = (0,0)
        self.buoy_found = False
        self.following_guide = False
        self.moved_to_point = False
        self.waypoint_rejected = False
        self.waypoint_aborted = False
        self.active_future_request = 0 # active future requests
        self.active_waypoint_request = 0 # keeps track of the number of active waypoint requests
        self.left_first = True # goes left of buoy first

        self.obstacles = None

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.blue_labels = set()
        self.red_labels = set()
        self.green_labels = set()

        self.declare_parameter(
            "color_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )

        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)


        if self.get_parameter("is_sim").get_parameter_value().bool_value:
            # hardcoded from reading YAML
            self.red_labels.add(label_mappings["red"])
            self.green_labels.add(label_mappings["green"])
            # self.blue_labels.add(label_mappings["green"])
            self.blue_labels.add(label_mappings["black"])
        else:
            self.declare_parameter(
                "buoy_label_mappings_file",
                os.path.join(
                    bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
                ),
            )
            buoy_label_mappings_file = self.get_parameter(
                "buoy_label_mappings_file"
            ).value
            with open(buoy_label_mappings_file, "r") as f:
                label_mappings = yaml.safe_load(f)
            for buoy_label in ["blue_buoy", "blue_circle", "blue_racquet_ball"]:
                self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["red_buoy", "red_circle", "red_racquet_ball"]:
                self.red_labels.add(label_mappings[buoy_label])
                self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["green_buoy", "green_circle"]:
                self.green_labels.add(label_mappings[buoy_label])
                # self.blue_labels.add(label_mappings[buoy_label])

        self.obstacles = []

        self.red_left = True
        self.gate_pair = None
        self.first_setup = True

    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.buoy_found = False
        self.runnerActivated = False
        self.following_guide = True
        self.moved_to_point = False
    
    def replace_closest(self, ref_obs, obstacles):
        if len(obstacles) == 0:
            return ref_obs, False
        opt_buoy = self.get_closest_to(self.ob_coords(ref_obs), obstacles)
        if self.norm(self.ob_coords(ref_obs), self.ob_coords(opt_buoy)) < self.duplicate_dist:
            return opt_buoy, True
        else:
            return ref_obs, False
        
    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
    def pair_angle_to_pose(self, pair, angle):
        quat = self.quaternion_from_euler(0, 0, angle)
        return Pose(
            position=Point(x=pair[0], y=pair[1]),
            orientation=Quaternion(x=quat[0], y=quat[2], z=quat[2], w=quat[3]),
        )

    def update_gate_wpt_pos(self, forward_dist = 0.0, tryhard=False):
        # split the buoys into red and green
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        self.gate_pair.left, res_left_left = self.replace_closest(self.gate_pair.left, red_buoys if self.red_left else green_buoys)
        self.gate_pair.right, res_right_right = self.replace_closest(self.gate_pair.right, green_buoys if self.red_left else red_buoys)
        if tryhard:
            _, res_left_right = self.replace_closest(self.gate_pair.left, green_buoys if self.red_left else red_buoys)
            _, res_right_left = self.replace_closest(self.gate_pair.right, red_buoys if self.red_left else green_buoys)
            # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint, it is false
            if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                self.get_logger().info('WE ARE GOING TO A FAKE PAIR, FIND PATH AGAIN')
                if not self.setup_buoys():
                    return self.gate_wpt
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        gate_wpt, self.buoy_direction = self.midpoint_pair_dir(self.gate_pair, forward_dist)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_angle_to_pose(
            pair=gate_wpt,
            # angle=(
            #     math.atan(self.ob_coords(pair.right)[1] - self.ob_coords(pair.left)[1]) /
            #     (self.ob_coords(pair.right)[0] - self.ob_coords(pair.left)[0])
            # ) + (math.pi / 2),
            angle=0,
        ), 0.0)]))
        return gate_wpt
        
    def execute_callback(self, goal_handle):
        self.start_process("Speed challenge task started!")

        self.reset_challenge()
        self.get_logger().info("Speed challenge setup completed.")

        # station keep logic
        # doesn't work?
        # self.get_logger().info(f"robot pose {self.robot_pos}")
        # self.move_to_point(self.robot_pos, is_stationary=True)

        success = False
        while not success:
            success = self.setup_buoys()
            time.sleep(self.timer_period)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()

        self.get_logger().info("Setup buoys succeeded!")

        self.gate_wpt, self.buoy_direction = self.midpoint_pair_dir(self.gate_pair, self.init_gate_dist)

        self.get_logger().info('going to the gate')
        while rclpy.ok():
            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", partial(self.update_gate_wpt_pos, -self.init_gate_dist)))
            break

        self.runnerActivated = True

        while rclpy.ok():
            # Check if the action client requested cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested. Aborting task initialization.")
                goal_handle.canceled()
                return Task.Result(success=False)

            if self.runnerActivated:
                self.home_pos = self.robot_pos # keep track of home position
                # self.buoy_direction = self.robot_dir

                self.get_logger().info(f"Facing direction: {self.buoy_direction}")
                task_result = self.run_actions()
                self.end_process("Speed challenge task ended.")
                return task_result
                
            time.sleep(self.timer_period)

        # If we exit the `while rclpy.ok()` loop somehow
        self.get_logger().info("ROS shutdown detected or loop ended unexpectedly.")
        goal_handle.abort()
        return Task.Result(success=False)

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

    def run_actions(self):
        '''
        Run all the actions, interrupted if an action fails
        '''
        actions = [self.probe_blue_buoy, self.circle_blue_buoy, self.return_to_start]
        for action in actions:
            action_result = action()
            if action_result.success == False:
                return action_result
        return Task.Result(success=True)
    
    def update_point(self, point_name, update_func):
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
            if (math.sqrt(dist_squared) > self.adaptive_distance):
                setattr(self, point_name, new_point)
                return True, new_point
            return False, None

    def probe_blue_buoy(self):
        '''
        Function to find the blue buoy by moving near it (general direction).
        Keeps on appending waypoints to the north/south until it finds 
        '''
        self.get_logger().info("Probing for blue buoy")
        max_guide_d = self.get_parameter("probe_distance").value
        current_guide_point = lambda: (max_guide_d*self.buoy_direction[0] + self.robot_pos[0], 
                        max_guide_d*self.buoy_direction[1] + self.robot_pos[1])
        self.guide_point = current_guide_point()
        self.get_logger().info(f"Current position: {self.robot_pos}. Guide point: {self.guide_point}.")
            
        # detection_success = self.move_to_point(self.guide_point, busy_wait=True,
        #                                         goal_update_func=partial(self.update_point, "guide_point", current_guide_point), 
        #                                         exit_func=self.blue_buoy_detected)
        detection_success = self.move_to_point(self.guide_point, busy_wait=True,
                                                exit_func=partial(self.blue_buoy_detected, True))
        if detection_success:
            return Task.Result(success=True)
        return Task.Result(success=False)

    def circle_blue_buoy(self):
        '''
        Function to circle the blue buoy.
        '''
        self.get_logger().info("Circling blue buoy")
        if not self.blue_buoy_detected():
            self.get_logger().info("speed challenge probing exited without finding blue buoy")
            return Task.Result(success=False)
        
        # circle the blue buoy like a baseball diamond
        # a better way to do this might be to have the astar run to original cell, 
        # but require the path to go around buoy

        t_o = self.get_parameter("turn_offset").get_parameter_value().double_value
        robot_x, robot_y = self.robot_pos
        robot_buoy_vector = (self.blue_buoy_pos[0]-robot_x, self.blue_buoy_pos[1]-robot_y)
        robot_buoy_dist = self.norm(robot_buoy_vector)
        self.buoy_direction = (robot_buoy_vector[0]/robot_buoy_dist, robot_buoy_vector[1]/robot_buoy_dist)
        first_dir = (self.buoy_direction[1]*t_o, -self.buoy_direction[0]*t_o)
        second_dir = (self.buoy_direction[0]*t_o, self.buoy_direction[1]*t_o)
        third_dir = (-first_dir[0], -first_dir[1])
        if not self.left_first:
            first_dir, third_dir = third_dir, first_dir

        add_tuple = lambda a,b: tuple(sum(x) for x in zip(a, b))
        first_base = add_tuple(self.blue_buoy_pos, first_dir)
        second_base = add_tuple(self.blue_buoy_pos, second_dir)
        third_base = add_tuple(self.blue_buoy_pos, third_dir)

        bases = [first_base, second_base, third_base]
        dirs = [first_dir, second_dir, third_dir]
        self.get_logger().info(f"initial moved to points= {self.moved_to_point}")
        self.get_logger().info(f"blue buoy pose: {self.blue_buoy_pos}")
        self.get_logger().info(f"bases: {bases}")
        for base, dir in zip(bases, dirs):
            def update_current_point():
                self.blue_buoy_detected()
                return add_tuple(self.blue_buoy_pos, dir)
            self.base_point = base
            self.move_to_point(self.base_point, busy_wait=True,
                               goal_update_func=partial(self.update_point, "base_point", update_current_point) )
            self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)
    
    def return_to_start(self):
        '''
        After circling the buoy, return to the starting position.
        '''
        self.get_logger().info("Returning to start")
        self.red_left = not self.red_left
        self.gate_pair.left, self.gate_pair.right = self.gate_pair.right, self.gate_pair.left
        self.gate_wpt, self.buoy_direction = self.midpoint_pair_dir(self.gate_pair, self.forward_dist_back)

        self.get_logger().info('going back to the gate')
        self.move_to_point(self.gate_wpt, busy_wait=True,
            goal_update_func=partial(self.update_point, "gate_wpt", partial(self.update_gate_wpt_pos, self.forward_dist_back)))
        # self.move_to_point(self.home_pos, busy_wait=True)
        return Task.Result(success=True)

    def _send_goal(self, goal_msg):
        self.follow_path_client.wait_for_server()
        self.active_future_request += 1
        self.send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg
        )
        self.send_goal_future.add_done_callback(self.follow_path_response_cb)

    def move_to_point(self, point, is_stationary=False, busy_wait=False, exit_func=None, goal_update_func=None):
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
                time.sleep(self.timer_period)
        return False

    def follow_path_response_cb(self, future):
        '''
        Responds to follow path action server goal response.
        '''
        goal_handle = future.result()
        self.active_future_request -= 1
        if self.active_future_request > 0:
            self.get_logger().info('PREVIOUS FUTURE, IGNORING')
            return
        
        if not goal_handle.accepted:
            self.get_logger().info('Waypoint rejected')
            self.waypoint_rejected = True
            return

        self.get_logger().info("Waypoint accepted")
        self._get_result_future = goal_handle.get_result_async()
        self.active_waypoint_request += 1
        self._get_result_future.add_done_callback(self.get_point_result_cb)

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

    def norm_squared(self, vec, ref=(0, 0)):
        return (vec[0] - ref[0])**2 + (vec[1]-ref[1])**2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))

    def blue_buoy_detected(self, buoy_front=False):
        '''
        Check if the blue buoy for turning is detected (returns boolean).
        Also sets the position of the blue buoy if it is found.
        '''
        backup_buoy = None
        updated_pos = False
        for obstacle in self.obstacles:
            if obstacle.label in self.blue_labels:
                buoy_dir = (obstacle.global_point.point.x-self.robot_pos[0], 
                            obstacle.global_point.point.y-self.robot_pos[1])
                dot_prod = buoy_dir[0] * self.robot_dir[0] + buoy_dir[1] * self.robot_dir[1]
                buoy_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
                if (backup_buoy is None) or (self.norm(self.blue_buoy_pos, buoy_pos) < self.norm(self.blue_buoy_pos, backup_buoy)):
                    backup_buoy = buoy_pos
                if ((not buoy_front) or (dot_prod > 0)) and ((not self.buoy_found) or (self.norm(self.blue_buoy_pos, buoy_pos) < self.duplicate_dist)): #check if buoy position is behind robot i.e. dot product is negative
                    self.buoy_found = True
                    updated_pos = True
                    self.blue_buoy_pos = buoy_pos
                    break
        if (not updated_pos) and (backup_buoy is not None):
            self.get_logger().info('SWITCHING TO BACKUP BUOY')
            self.blue_buoy_pos = backup_buoy
        return self.buoy_found
    
    def ob_coords(self, buoy, local=False):
        if local:
            return (buoy.local_point.point.x, buoy.local_point.point.y)
        else:
            return (buoy.global_point.point.x, buoy.global_point.point.y)

    def get_closest_to(self, source, buoys, local=False):
        return min(
            buoys,
            key=lambda buoy: math.dist(source, self.ob_coords(buoy, local)),
        )

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def midpoint_pair_dir(self, pair, forward_dist):
        left_coords = self.ob_coords(pair.left)
        right_coords = self.ob_coords(pair.right)
        midpoint = self.midpoint(left_coords, right_coords)
        
        scale = forward_dist
        dy = right_coords[1] - left_coords[1]
        dx = right_coords[0] - left_coords[0]
        norm = math.sqrt(dx**2 + dy**2)
        dx /= norm
        dy /= norm
        midpoint = (midpoint[0] - scale*dy, midpoint[1] + scale*dx)

        return midpoint, (-dy, dx)

    def split_buoys(self, obstacles):
        """
        Splits the buoys into red and green based on their labels in the obstacle map
        """
        green_bouy_points = []
        red_bouy_points = []
        for obstacle in obstacles:
            if obstacle.label in self.green_labels:
                green_bouy_points.append(obstacle)
            elif obstacle.label in self.red_labels:
                red_bouy_points.append(obstacle)
        return green_bouy_points, red_bouy_points

    def obs_to_pos(self, obs):
        return [self.ob_coords(ob, local=False) for ob in obs]

    def obs_to_pos_label(self, obs):
        return [self.ob_coords(ob, local=False) + (ob.label,) for ob in obs]

    def buoy_pairs_to_markers(self, buoy_pairs):
        """
        Create the markers from an array of buoy pairs to visualize them (and the respective waypoints) in RViz
        """
        marker_array = MarkerArray()
        i = 0
        for p_left, p_right, point, radius in buoy_pairs:
            marker_array.markers.append(
                Marker(
                    type=Marker.ARROW,
                    pose=point,
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=2.0, y=0.15, z=0.15),
                    color=ColorRGBA(a=1.0, b=1.0),
                    id=(4 * i),
                )
            )
            if self.red_left:
                left_color = ColorRGBA(r=1.0, a=1.0)
                right_color = ColorRGBA(g=1.0, a=1.0)
            else:
                left_color = ColorRGBA(g=1.0, a=1.0)
                right_color = ColorRGBA(r=1.0, a=1.0)

            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(p_left)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=left_color,
                    id=(4 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(p_right)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=right_color,
                    id=(4 * i) + 2,
                )
            )
            i += 1
        return marker_array

    
    def setup_buoys(self):
        """
        Runs when the first obstacle map is received, filters the buoys that are in front of
        the robot (x>0 in local coordinates) and finds (and stores) the closest green one and
        the closest red one, and because the robot is in the starting position these
        are the front buoys of the robot starting box.
        """
        self.get_logger().debug("Setting up starting buoys!")
        self.get_logger().debug(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )

        # Split all the buoys into red and green
        green_init, red_init = self.split_buoys(self.obstacles)

        # lambda function that filters the buoys that are in front of the robot
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            if ob.local_point.point.x > 0
        ]
        # take the green and red buoys that are in front of the robot
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().debug(
            f"initial red buoys: {[self.ob_coords(buoy) for buoy in red_buoys]}, green buoys: {[self.ob_coords(buoy) for buoy in green_buoys]}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            self.get_logger().debug("No starting buoy pairs!")
            return False

        # From the red buoys that are in front of the robot, take the one that is closest to it.
        # And do the same for the green buoys.
        # This pair is the front pair of the starting box of the robot.
        # want to pick the pair that's far apart but has the closest midpoint
        if self.first_setup:
            green_to = None
            red_to = None
            for red_b in red_buoys:
                for green_b in green_buoys:
                    if self.norm(self.ob_coords(red_b), self.ob_coords(green_b)) < self.inter_buoy_pair_dist:
                        continue
                    elif (green_to is None) or (self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True))) < self.norm(self.midpoint(self.ob_coords(red_to, local=True), self.ob_coords(green_to, local=True)))):
                        green_to = green_b
                        red_to = red_b
            if green_to is None:
                return False
            if self.ccw((0, 0), self.ob_coords(green_to, local=True), self.ob_coords(red_to, local=True)):
                self.red_left = True
                self.gate_pair = InternalBuoyPair(red_to, green_to)
                self.get_logger().debug("RED BUOYS LEFT, GREEN BUOYS RIGHT")
            else:
                self.red_left = False
                self.gate_pair = InternalBuoyPair(green_to, red_to)
                self.get_logger().debug("GREEN BUOYS LEFT, RED BUOYS RIGHT")
            self.first_setup = False
            return True
        else:
            green_to = None
            red_to = None
            for red_b in red_buoys:
                for green_b in green_buoys:
                    if self.norm(self.ob_coords(red_b), self.ob_coords(green_b)) < self.inter_buoy_pair_dist:
                        continue
                    elif ((green_to is None) or (self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True))) < self.norm(self.midpoint(self.ob_coords(red_to, local=True), self.ob_coords(green_to, local=True))))) and (self.red_left == self.ccw((0, 0), self.ob_coords(green_b, local=True), self.ob_coords(red_b, local=True))):
                        green_to = green_b
                        red_to = red_b
            if green_to is None:
                return False
            if self.red_left:
                self.gate_pair = InternalBuoyPair(red_to, green_to)
            else:
                self.gate_pair = InternalBuoyPair(green_to, red_to)
            return True

    def ccw(self, a, b, c):
        """Return True if the points a, b, c are counterclockwise, respectively"""
        area = (
            a[0] * b[1]
            + b[0] * c[1]
            + c[0] * a[1]
            - a[1] * b[0]
            - b[1] * c[0]
            - c[1] * a[0]
        )
        return area > 0

    def filter_front_buoys(self, pair, buoys):
        """
        Returns the buoys (from the given array) that are in front of a pair of points,
        considering the forward direction to be the one such that
        the first point of the pair is in the left and the second is in the right
        """
        # (red, green)
        return [
            buoy
            for buoy in buoys
            if (self.ccw(
                self.ob_coords(pair.left),
                self.ob_coords(pair.right),
                self.ob_coords(buoy),
            ) and (min(self.norm(self.ob_coords(buoy), self.ob_coords(pair.left)), self.norm(self.ob_coords(buoy), self.ob_coords(pair.right))) > self.buoy_pair_dist_thres))
        ]

    def pick_buoy(self, buoys, prev_mid, ref_buoy):
        # sort by distance
        buoys.sort(key=lambda buoy: self.norm(prev_mid, self.ob_coords(buoy)))
        for buoy in buoys:
            if self.norm(self.ob_coords(ref_buoy), self.ob_coords(buoy)) > self.duplicate_dist:
                return False, buoy
        return True, ref_buoy



def main(args=None):
    rclpy.init(args=args)
    node = SpeedChallenge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
