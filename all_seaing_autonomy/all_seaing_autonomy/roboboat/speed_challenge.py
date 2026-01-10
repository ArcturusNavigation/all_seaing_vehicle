#!/usr/bin/env python3
from ast import Num
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.msg import ObstacleMap, Obstacle, ControlOption
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from all_seaing_common.task_server_base import TaskServerBase

import os
import yaml
import math
import time
from collections import deque
from functools import partial
from enum import Enum

TIMER_PERIOD = 1 / 60

class SpeedChallengeState(Enum):
    SETTING_UP = 1
    GATES = 2
    PROBING_BUOY = 3
    CIRCLING = 4
    RETURNING = 5

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

class SpeedChallenge(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "speed_challenge_server", action_name = "speed_challenge")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/labeled", self.map_cb, 10
        )

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("gate_dist_thres", 40.0)
        self.gate_dist_thres = self.get_parameter("gate_dist_thres").get_parameter_value().double_value

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

        self.declare_parameter("exit_turn_eps", 0.4) #roughly a bit less than pi/6 both ways
        self.exit_turn_eps = self.get_parameter("exit_turn_eps").get_parameter_value().double_value

        self.declare_parameter("t_o_eps", 0.5)
        self.t_o_eps = self.get_parameter("t_o_eps").get_parameter_value().double_value

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

        self.declare_parameter("is_sim", False)
        self.declare_parameter("turn_offset", 5.0)

        self.blue_buoy_pos = (0, 0)
        
        self.buoy_direction = (0,0)
        self.buoy_found = False
        self.following_guide = False
        self.left_first = True # goes left of buoy first

        self.obstacles = None

        self.state = SpeedChallengeState.SETTING_UP

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
            self.blue_labels.add(label_mappings["yellow"])
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
                # self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["green_buoy", "green_circle"]:
                self.green_labels.add(label_mappings[buoy_label])
                # self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["yellow_buoy", "yellow_racquet_ball"]:
                self.blue_labels.add(label_mappings[buoy_label])

        self.obstacles = []

        self.red_left = True
        self.gate_pair = None
        self.first_setup = True

    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.buoy_found = False
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
    
    def should_accept_task(self, goal_request):
        if self.obstacles is None:
            return False
        self.first_setup = True
        return self.setup_buoys()
    
    # def init_setup(self):
    #     if self.obstacles is None:
    #         return
    #     success = self.setup_buoys()
    #     if success:
    #         self.get_logger().info("Setup buoys succeeded!")
    #         self.state = SpeedChallengeState.GATES
    #         self.mark_successful()

    def init_setup(self):
        self.get_logger().info("Setup buoys succeeded!")
        self.state = SpeedChallengeState.GATES
        self.mark_successful()

    def control_loop(self):
        action_result = Task.Result(success=True)
        if self.state == SpeedChallengeState.RETURNING:
            action_result = self.return_to_start()
            if action_result.success:
                self.mark_successful()
            else:
                self.mark_unsuccessful()
        elif self.state == SpeedChallengeState.CIRCLING:
            action_result = self.smooth_circle_blue_buoy()
            self.state = SpeedChallengeState.RETURNING
        elif self.state == SpeedChallengeState.PROBING_BUOY:
            action_result = self.probe_blue_buoy()
            self.state = SpeedChallengeState.CIRCLING
        elif self.state == SpeedChallengeState.GATES:
            self.gate_wpt, self.buoy_direction = self.midpoint_pair_dir(self.gate_pair, self.init_gate_dist)

            self.get_logger().info('going behind the gate')

            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, -self.init_gate_dist)))
            
            self.get_logger().info('going in front of the gate')
            
            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.init_gate_dist)))
            
            self.state = SpeedChallengeState.PROBING_BUOY
        
        # cancel control loop if a single action fails
        if not action_result.success:
            self.mark_unsuccessful()
        

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

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
                               goal_update_func=partial(self.update_point, "base_point", self.adaptive_distance, update_current_point) )
            self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)

    def smooth_circle_blue_buoy(self):
        '''
        Function to circle the blue buoy in a smooth fashion.
        '''
        self.get_logger().info("Circling blue buoy")
        if not self.blue_buoy_detected():
            self.get_logger().info("speed challenge probing exited without finding blue buoy")
            return Task.Result(success=False)

        t_o = self.get_parameter("turn_offset").get_parameter_value().double_value
        robot_x, robot_y = self.robot_pos
        robot_buoy_vector = (self.blue_buoy_pos[0]-robot_x, self.blue_buoy_pos[1]-robot_y)
        robot_buoy_dist = self.norm(robot_buoy_vector)
        self.buoy_direction = (robot_buoy_vector[0]/robot_buoy_dist, robot_buoy_vector[1]/robot_buoy_dist)
        first_dir = (self.buoy_direction[1]*(t_o+self.t_o_eps), -self.buoy_direction[0]*(t_o+self.t_o_eps))
        if not self.left_first:
            first_dir = (-first_dir[0], -first_dir[1])

        add_tuple = lambda a,b: tuple(sum(x) for x in zip(a, b))
        self.first_base = add_tuple(self.blue_buoy_pos, first_dir)

        self.get_logger().info(f"initial moved to points= {self.moved_to_point}")
        self.get_logger().info(f"blue buoy pose: {self.blue_buoy_pos}")
        self.get_logger().info(f"first base: {self.first_base}")

        def update_first_base():
            self.blue_buoy_detected()
            return add_tuple(self.blue_buoy_pos, first_dir)
        self.move_to_point(self.first_base, busy_wait=True,
                            goal_update_func=partial(self.update_point, "first_base", self.adaptive_distance, update_first_base) )
        self.get_logger().info(f"moved to first base = {self.moved_to_point}")

        in_circling = False # boolean flag for whether boat is circling, set to True when boat has turned at least 90 degrees
        def exit_angle_met():
            cur_robot_dir = self.robot_dir
            buoy_gate_vector =  (self.gate_wpt[0]-self.blue_buoy_pos[0],self.gate_wpt[1] -self.blue_buoy_pos[1])
            buoy_gate_dir = (buoy_gate_vector[0]/self.norm(buoy_gate_vector), buoy_gate_vector[1]/self.norm(buoy_gate_vector))
            angle = math.atan2(cur_robot_dir[1], cur_robot_dir[0]) - math.atan2(buoy_gate_dir[1], buoy_gate_dir[0])
            if (angle < 0):
                angle += 2*math.pi
            return (angle < self.exit_turn_eps) or (angle > 2*math.pi-self.exit_turn_eps)
        
        self.turn_pid.reset()
        self.turn_pid.set_setpoint(t_o)
        self.turn_pid.set_effort_max(self.max_turn_vel[2])
        self.turn_pid.set_effort_min(-self.max_turn_vel[2])
        self.prev_update_time = self.get_clock().now()
        self.get_logger().info(f"Circling buoy via PID")
        while (not in_circling) or (not exit_angle_met()):
            pid_output = self.turn_pid.get_effort()
            # send velocity commands
            self.send_vel_cmd(self.max_turn_vel[0], 0.0, (-1.0 if self.left_first else 1.0)*pid_output)
            # get feedback
            self.blue_buoy_detected()
            dist_to_buoy = self.norm(self.blue_buoy_pos, self.robot_pos)
            dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9

            self.get_logger().info(f"PID values: set_point {t_o}, effort {pid_output:.3f}, sense value {dist_to_buoy:.3f}")
            self.turn_pid.update(dist_to_buoy,dt)

            # TODO: update in_Circling correctly (check 90 degrees)
            in_circling = True
            time.sleep(TIMER_PERIOD)
        self.get_logger().info(f"Finished circling buoy")
        return Task.Result(success=True)
    
    def return_to_start(self):
        '''
        After circling the buoy, return to the starting position.
        '''
        self.get_logger().info("Returning to start")
        self.red_left = not self.red_left
        self.gate_pair.left, self.gate_pair.right = self.gate_pair.right, self.gate_pair.left
        # make the robot face the previous gate
        # _, intended_dir = self.midpoint_pair_dir(self.gate_pair, 0.0)
        # theta_intended = math.atan2(intended_dir[1], intended_dir[0])
        # nav_x, nav_y = self.robot_pos
        # self.move_to_waypoint([nav_x, nav_y, theta_intended], is_stationary=False, busy_wait=True, cancel_on_exit=True)
        # recompute gate
        # self.setup_buoys()
        gate_mid, _ = self.midpoint_pair_dir(self.gate_pair, 0.0)
        self.setup_buoys(self.difference(self.robot_pos, gate_mid))
        self.gate_wpt, self.buoy_direction = self.midpoint_pair_dir(self.gate_pair, self.forward_dist_back)

        self.get_logger().info('going back to the gate')
        self.move_to_point(self.gate_wpt, busy_wait=True,
            goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, -self.forward_dist_back)))
        self.move_to_point(self.gate_wpt, busy_wait=True,
            goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.forward_dist_back)))
        return Task.Result(success=True)

    def norm_squared(self, vec, ref=(0, 0)):
        return (vec[0] - ref[0])**2 + (vec[1]-ref[1])**2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))
    
    def dot(self, vec1, vec2):
        return vec1[0]*vec2[0]+vec1[1]*vec2[1]
    
    def difference(self, pt1, pt2):
        """
        pt2 - pt1
        """
        x1, y1 = pt1
        x2, y2 = pt2
        return (x2-x1, y2-y1)

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
                if (backup_buoy is None) or (self.buoy_found and (self.norm(self.blue_buoy_pos, buoy_pos) < self.norm(self.blue_buoy_pos, backup_buoy))):
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

    
    def setup_buoys(self, pointing_direction=None):
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
        # lambda function that filters the buoys that are in front of the robot
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            if ((pointing_direction is None and ob.local_point.point.x > 0) or (pointing_direction is not None and self.dot(self.difference(self.robot_pos, self.ob_coords(ob)), pointing_direction) > 0)) and self.norm(self.robot_pos, self.ob_coords(ob)) < self.gate_dist_thres
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
            self.get_logger().info(f'FOUND GATE')
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
