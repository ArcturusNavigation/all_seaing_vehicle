#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor

from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import Task
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from all_seaing_common.task_server_base import TaskServerBase

from all_seaing_common.report_pb2 import GatePass, GateType
from tf_transformations import quaternion_from_euler
from all_seaing_autonomy.geometry_utils import ccw
from all_seaing_autonomy.buoy_utils import (
    InternalBuoyPair, ob_coords, get_closest_to, midpoint_pair_dir,
    split_buoys, obs_to_pos, obs_to_pos_label, filter_front_buoys,
    pick_buoy, replace_closest, buoy_pairs_distance, buoy_pairs_angle,
    get_acute_angle, get_triangle_angle, check_better_pair_angles,
    better_buoy_pair_transition, check_better_one_side, buoy_pairs_to_markers, pair_to_pose
)

import os
import yaml
import math
import numpy as np
import time
from collections import deque
from functools import partial
from enum import Enum

TIMER_PERIOD = 1 / 60

class ReturnState(Enum):
    SETTING_UP = 1
    ENTERING = 2

class EntryGates(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "entry_gates_server", action_name = "entry_gates", search_action_name = "search_entry")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("max_gate_pair_dist", 25.0)
        self.max_gate_pair_dist = self.get_parameter("max_gate_pair_dist").get_parameter_value().double_value

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.declare_parameter("better_angle_thres", 0.2)
        self.better_angle_thres = self.get_parameter("better_angle_thres").get_parameter_value().double_value

        self.declare_parameter("gate_dist_back", 1.0)
        self.forward_dist_back = self.get_parameter("gate_dist_back").get_parameter_value().double_value

        self.declare_parameter("gate_probe_dist", 1.0)
        self.gate_probe_dist = self.get_parameter("gate_probe_dist").get_parameter_value().double_value

        self.obstacles = None

        self.state = ReturnState.SETTING_UP

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

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
        # hardcoded from reading YAML
        if self.is_sim:
            self.green_labels.add(label_mappings["green"])
            self.red_labels.add(label_mappings["red"])
        else:
            # self.green_labels.add(11) # just to use old rosbags
            # self.red_labels.add(17) # just to use old rosbags
            # self.green_labels.add(label_mappings["green_buoy"])
            # self.green_labels.add(label_mappings["green_circle"])
            self.green_labels.add(label_mappings["green_pole_buoy"])
            # self.red_labels.add(label_mappings["red_buoy"])
            # self.red_labels.add(label_mappings["red_circle"])
            self.red_labels.add(label_mappings["red_pole_buoy"])
            # self.red_labels.add(label_mappings["yellow_buoy"])
            # self.red_labels.add(label_mappings["yellow_racquet_ball"])
        self.declare_parameter(
            "task_locations_file",
            os.path.join(
                bringup_prefix, "config", "course", "task_locations.yaml"
            ),
        )

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

    def should_accept_task(self, goal_request):
        if self.obstacles is None:
            return False
        return self.setup_buoys(None, filter_front=True, visualize=True)

    def init_setup(self):
        self.get_logger().info("Setup buoys succeeded!")
        self.state = ReturnState.ENTERING
        self.mark_successful()

    def control_loop(self):
        self.enter_course()
        self.mark_successful()


    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

    def enter_course(self):
        self.get_logger().info("Entering the course")
        self.new_gate_pair = self.gate_pair
        while self.new_gate_pair is not None:
            self.gate_pair = self.new_gate_pair
            self.gate_wpt, _ = midpoint_pair_dir(self.gate_pair, -self.forward_dist_back)
            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, self.duplicate_dist, -self.forward_dist_back)))
            self.new_gate_pair = None
            self.gate_wpt, _ = midpoint_pair_dir(self.gate_pair, self.gate_probe_dist)
            self.move_to_point(self.gate_wpt, busy_wait=True,
                exit_func=partial(self.next_pair, self.gate_pair), goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.obstacles, self.green_labels, self.red_labels, self.duplicate_dist, self.gate_probe_dist)))
        self.report_data(GatePass(
            type=GateType.GATE_ENTRY,
            position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

        return Task.Result(success=True)

    def next_pair(self, prev_pair):
        green, red = split_buoys(self.obstacles, self.green_labels, self.red_labels)
        front_red = filter_front_buoys(prev_pair, red, self.buoy_pair_dist_thres)
        front_green = filter_front_buoys(prev_pair, green, self.buoy_pair_dist_thres)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, front red buoys: {obs_to_pos(front_red)}, front green buoys: {obs_to_pos(front_green)}"
        )

        if ((not front_red) and (not front_green)) or ((prev_pair is None) and ((not front_red) or (not front_green))):
            prev_coords = ob_coords(prev_pair.left), ob_coords(
                prev_pair.right
            )
            red_coords = obs_to_pos(red)
            green_coords = obs_to_pos(green)

            self.get_logger().debug(
                f"buoys:  {prev_coords} \nred: {red_coords} \ngreen: {green_coords}"
            )
            self.get_logger().debug(f"robot: {self.robot_pos}")
            self.get_logger().debug("Missing at least one front buoy")
            return False

        prev_pair_midpoint, _ = midpoint_pair_dir(prev_pair, 0.0)
        self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
        left_duplicate, left_next = pick_buoy(front_red if self.red_left else front_green, prev_pair_midpoint, prev_pair.left, self.duplicate_dist)
        right_duplicate, right_next = pick_buoy(front_green if self.red_left else front_red, prev_pair_midpoint, prev_pair.right, self.duplicate_dist)

        improved = False
        for left_buoy in ((True, prev_pair.left), (left_duplicate, left_next)):
            for right_buoy in ((True, prev_pair.right), (right_duplicate, right_next)):
                if left_buoy[0] and right_buoy[0]:
                    continue

                if left_buoy[0] and (not check_better_one_side(prev_pair.left, prev_pair.right, right_buoy[1], self.better_angle_thres)):
                    continue

                if right_buoy[0] and (not check_better_one_side(prev_pair.right, prev_pair.left, left_buoy[1], self.better_angle_thres)):
                    continue

                if np.linalg.norm(ob_coords(left_buoy[1]) - ob_coords(right_buoy[1])) < self.inter_buoy_pair_dist or np.linalg.norm(ob_coords(left_buoy[1]) - ob_coords(right_buoy[1])) > self.max_inter_gate_dist:
                    continue

                cur = InternalBuoyPair(left_buoy[1], right_buoy[1])

                if buoy_pairs_distance(prev_pair, cur, "mid") > self.max_gate_pair_dist:
                    continue

                if (self.new_gate_pair is None) or better_buoy_pair_transition(self.new_gate_pair, cur, prev_pair, self.buoy_pair_dist_thres, self.better_angle_thres, self.duplicate_dist):
                    self.new_gate_pair = cur
                    improved = True
        return improved



def main(args=None):
    rclpy.init(args=args)
    node = EntryGates()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
