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
from all_seaing_autonomy.geometry_utils import ccw, quaternion_from_euler
from all_seaing_autonomy.buoy_utils import (
    InternalBuoyPair, ob_coords, get_closest_to, midpoint_pair_dir,
    split_buoys, obs_to_pos, obs_to_pos_label, filter_front_buoys,
    pick_buoy, replace_closest, buoy_pairs_distance, buoy_pairs_angle,
    get_acute_angle, get_triangle_angle, check_better_pair_angles,
    better_buoy_pair_transition, check_better_one_side,
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

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter("red_left", True)
        self.red_left = self.get_parameter("red_left").get_parameter_value().bool_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("gate_dist_thres", 50.0)
        self.gate_dist_thres = self.get_parameter("gate_dist_thres").get_parameter_value().double_value

        self.declare_parameter("circling_buoy_dist_thres", 25.0)
        self.circling_buoy_dist_thres = self.get_parameter("circling_buoy_dist_thres").get_parameter_value().double_value

        self.declare_parameter("max_inter_gate_dist", 25.0)
        self.max_inter_gate_dist = self.get_parameter("max_inter_gate_dist").get_parameter_value().double_value

        self.declare_parameter("max_gate_pair_dist", 25.0)
        self.max_gate_pair_dist = self.get_parameter("max_gate_pair_dist").get_parameter_value().double_value

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.declare_parameter("inter_buoy_pair_dist", 1.0)
        self.inter_buoy_pair_dist = self.get_parameter("inter_buoy_pair_dist").get_parameter_value().double_value

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

        self.gate_pair = None

    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))

    def update_gate_wpt_pos(self, forward_dist = 0.0, tryhard=False):
        green_buoys, red_buoys = split_buoys(self.obstacles, self.green_labels, self.red_labels)
        self.gate_pair.left, res_left_left = replace_closest(self.gate_pair.left, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
        self.gate_pair.right, res_right_right = replace_closest(self.gate_pair.right, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
        if tryhard:
            _, res_left_right = replace_closest(self.gate_pair.left, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
            _, res_right_left = replace_closest(self.gate_pair.right, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
            if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                self.get_logger().info('WE ARE GOING TO A FAKE PAIR, FIND PATH AGAIN')
                if not self.setup_buoys():
                    return self.gate_wpt
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        gate_wpt, self.buoy_direction = midpoint_pair_dir(self.gate_pair, forward_dist)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(gate_wpt), 0.0)]))
        return gate_wpt

    def should_accept_task(self, goal_request):
        if self.obstacles is None:
            return False
        return self.setup_buoys()

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
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, -self.forward_dist_back)))
            self.new_gate_pair = None
            self.gate_wpt, _ = midpoint_pair_dir(self.gate_pair, self.gate_probe_dist)
            self.move_to_point(self.gate_wpt, busy_wait=True,
                exit_func=partial(self.next_pair, self.gate_pair), goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.gate_probe_dist)))

        self.report_data(GatePass(
            type=GateType.GATE_ENTRY,
            position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

        return Task.Result(success=True)

    def buoy_pairs_to_markers(self, buoy_pairs):
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
                    pose=self.pair_to_pose(ob_coords(p_left)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=left_color,
                    id=(4 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(ob_coords(p_right)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=right_color,
                    id=(4 * i) + 2,
                )
            )
            i += 1
        return marker_array


    def setup_buoys(self, ref_pair = None):
        self.get_logger().debug("Setting up starting buoys!")
        self.get_logger().debug(
            f"list of obstacles: {obs_to_pos_label(self.obstacles)}"
        )

        green_init, red_init = split_buoys(self.obstacles, self.green_labels, self.red_labels)

        robot_pos = np.array(self.robot_pos)
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            if ob.local_point.point.x > 0 and np.linalg.norm(robot_pos - ob_coords(ob)) < self.gate_dist_thres
        ]
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().debug(
            f"initial red buoys: {[ob_coords(buoy) for buoy in red_buoys]}, green buoys: {[ob_coords(buoy) for buoy in green_buoys]}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            self.get_logger().debug("No starting buoy pairs!")
            return False

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
        self.get_logger().info(f'FOUND GATE')
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        self.gate_mid, self.gate_dir = midpoint_pair_dir(self.gate_pair, 0.0)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(self.gate_mid), 0.0)]))
        return True

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
