#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import Task
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from all_seaing_common.task_server_base import TaskServerBase

from all_seaing_common.report_pb2 import ObjectDetected, ObjectType, Color, TaskType, GatePass, GateType
from tf_transformations import quaternion_from_euler
from all_seaing_autonomy.geometry_utils import ccw
from all_seaing_autonomy.buoy_utils import (InternalBuoyPair, ob_coords, get_closest_to, midpoint_pair_dir, split_buoys, obs_to_pos, obs_to_pos_label, filter_front_buoys, pick_buoy, replace_closest, buoy_pairs_distance, buoy_pairs_angle, get_acute_angle, get_triangle_angle, check_better_pair_angles, better_buoy_pair_transition, check_better_one_side)

import os
import yaml
import math
import numpy as np
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

class SpeedChallenge(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "speed_challenge_server", action_name = "speed_challenge", search_action_name = "search_speed")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.declare_parameter("beacon_dist_thres", 15.0)
        self.beacon_dist_thres = self.get_parameter("beacon_dist_thres").get_parameter_value().double_value

        self.declare_parameter("probe_distance", 10.0)
        self.probe_distance = self.get_parameter("probe_distance").get_parameter_value().double_value

        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("init_gate_dist", 1.0)
        self.init_gate_dist = self.get_parameter("init_gate_dist").get_parameter_value().double_value

        self.declare_parameter("gate_dist_back", 1.0)
        self.forward_dist_back = self.get_parameter("gate_dist_back").get_parameter_value().double_value

        self.buoy_direction = np.array([0.0, 0.0])
        self.following_guide = False
        self.left_first = False # goes left of buoy first
        self.temp_left_first = self.left_first

        self.obstacles = None

        self.state = SpeedChallengeState.SETTING_UP

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.blue_labels = set()
        self.red_labels = set()
        self.green_labels = set()
        self.red_beacon_labels = set()
        self.green_beacon_labels = set()

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
            # self.declare_parameter(
            #     "buoy_label_mappings_file",
            #     os.path.join(
            #         bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
            #     ),
            # )
            # buoy_label_mappings_file = self.get_parameter(
            #     "buoy_label_mappings_file"
            # ).value
            # with open(buoy_label_mappings_file, "r") as f:
            #     label_mappings = yaml.safe_load(f)
            # for buoy_label in ["blue_buoy", "blue_circle", "blue_racquet_ball"]:
            #     self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["red_buoy", "red_circle", "red_racquet_ball"]:
                self.red_labels.add(label_mappings[buoy_label])
                # self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["green_buoy", "green_circle"]:
                self.green_labels.add(label_mappings[buoy_label])
                # self.blue_labels.add(label_mappings[buoy_label])
            for buoy_label in ["yellow_buoy", "yellow_racquet_ball"]:
                self.blue_labels.add(label_mappings[buoy_label])

            self.green_beacon_labels.add(label_mappings["green_indicator"])
            self.red_beacon_labels.add(label_mappings["red_indicator"])

        self.declare_parameter(
            "latlng_locations_file",
            os.path.join(
                bringup_prefix, "config", "localization", "locations.yaml"
            ),
        )

        with open(self.get_parameter("latlng_locations_file").value, "r") as f:
            self.latlng_location_mappings = yaml.safe_load(f)

        self.declare_parameter("location", "nbpark")
        self.location = self.get_parameter("location").get_parameter_value().string_value

        self.latlng_origin = self.latlng_location_mappings[self.location]

        # self.first_setup = True

    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.circling_buoy_found = False
        self.following_guide = True
        self.moved_to_point = False

    def should_accept_task(self, goal_request):
        if self.obstacles is None:
            return False
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
        self.turn_pid.reset()
        self.reset_challenge()
        self.mark_successful()

    def control_loop(self):
        action_result = Task.Result(success=True)
        if self.state == SpeedChallengeState.RETURNING:
            action_result = self.return_to_start()

            self.report_data(GatePass(
                type=GateType.GATE_SPEED_END,
                position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

            if action_result.success:
                self.mark_successful()
            else:
                self.mark_unsuccessful()
        elif self.state == SpeedChallengeState.CIRCLING:
            self.left_first = self.temp_left_first
            self.get_logger().info('LEFT FIRST' if self.left_first else 'RIGHT FIRST')
            action_result = self.smooth_circle_buoy(self.gate_wpt, self.adaptive_distance, self.duplicate_dist, self.gate_wpt, 1, self.left_first)
            self.state = SpeedChallengeState.RETURNING
        elif self.state == SpeedChallengeState.PROBING_BUOY:
            action_result = self.probe_buoy(self.buoy_direction, self.probe_distance, partial(self.buoy_detected, self.obstacles, self.blue_labels, self.duplicate_dist))
            self.state = SpeedChallengeState.CIRCLING
        elif self.state == SpeedChallengeState.GATES:
            self.gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, self.init_gate_dist)

            self.get_logger().info('going behind the gate')

            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, -self.init_gate_dist)))

            if self.goal_handle.is_cancel_requested:
                self.mark_unsuccessful()
                return

            self.get_logger().info('going in front of the gate')

            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, self.init_gate_dist)))

            if self.goal_handle.is_cancel_requested:
                self.mark_unsuccessful()
                return

            self.report_data(GatePass(
                type=GateType.GATE_SPEED_START,
                position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

            self.state = SpeedChallengeState.PROBING_BUOY

        # cancel control loop if a single action fails
        if not action_result.success:
            self.mark_unsuccessful()

    def identify_beacon(self):
        for obstacle in self.obstacles:
            if np.linalg.norm(self.robot_pos - ob_coords(obstacle)) > self.beacon_dist_thres:
                continue
            # below might toggle some times but will hopefully settle before we start circling, it's fixed once we start circling
            if obstacle.label in self.red_beacon_labels:
                self.temp_left_first = False
                self.report_data(ObjectDetected(
                    object_type=ObjectType.OBJECT_LIGHT_BEACON,
                    color=Color.COLOR_RED,
                    position=self.pos_to_latlng(self.latlng_origin, ob_coords(obstacle)),
                    object_id=obstacle.id,
                    task_context=TaskType.TASK_SPEED_CHALLENGE))
            elif obstacle.label in self.green_beacon_labels:
                self.temp_left_first = True
                self.report_data(ObjectDetected(
                    object_type=ObjectType.OBJECT_LIGHT_BEACON,
                    color=Color.COLOR_GREEN,
                    position=self.pos_to_latlng(self.latlng_origin, ob_coords(obstacle)),
                    object_id=obstacle.id,
                    task_context=TaskType.TASK_SPEED_CHALLENGE))

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles
        if self.paused:
            return
        # self.identify_beacon()

    def return_to_start(self):
        '''
        After circling the buoy, return to the starting position.
        '''
        self.get_logger().info("Returning to start")
        self.red_left = not self.red_left
        self.gate_pair.left, self.gate_pair.right = self.gate_pair.right, self.gate_pair.left
        # make the robot face the previous gate
        # _, intended_dir = midpoint_pair_dir(self.gate_pair, 0.0)
        # theta_intended = math.atan2(intended_dir[1], intended_dir[0])
        # nav_x, nav_y = self.robot_pos
        # self.move_to_waypoint([nav_x, nav_y, theta_intended], is_stationary=False, busy_wait=True, cancel_on_exit=True)
        # recompute gate
        # self.setup_buoys()
        # gate_mid, _ = midpoint_pair_dir(self.gate_pair, 0.0)
        # self.setup_buoys(self.difference(self.robot_pos, gate_mid))
        self.get_logger().info('probing & recomputing gate')
        probing_wpt, _ = midpoint_pair_dir(self.gate_pair, -self.forward_dist_back)
        self.move_to_point(probing_wpt, busy_wait=True, exit_func=self.setup_buoys)

        self.get_logger().info('going back to the gate')
        self.gate_wpt, _ = midpoint_pair_dir(self.gate_pair, -self.forward_dist_back) # gate detected after probing or the one computed at the start
        self.move_to_point(self.gate_wpt, busy_wait=True,
            goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, -self.forward_dist_back)))
        self.gate_wpt, _ = midpoint_pair_dir(self.gate_pair, self.forward_dist_back)
        self.move_to_point(self.gate_wpt, busy_wait=True,
            goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, self.forward_dist_back)))
        return Task.Result(success=True)


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
