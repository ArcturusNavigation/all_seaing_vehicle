#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import Task
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.task_server_base import TaskServerBase
from all_seaing_controller.pid_controller import PIDController
from tf_transformations import quaternion_from_euler
from all_seaing_common.report_pb2 import ObjectDetected, ObjectType, Color, TaskType
from all_seaing_autonomy.geometry_utils import ccw
from all_seaing_autonomy.buoy_utils import (InternalBuoyPair, ob_coords, get_closest_to, midpoint_pair_dir, split_buoys, obs_to_pos, obs_to_pos_label, filter_front_buoys, pick_buoy, replace_closest, buoy_pairs_distance, buoy_pairs_angle, get_acute_angle, get_triangle_angle, check_better_pair_angles, better_buoy_pair_transition, check_better_one_side, buoy_pairs_to_markers, pair_angle_to_pose)

import math
import numpy as np
import os
import yaml
import time
from enum import Enum
from functools import partial

class FollowPathState(Enum):
    SETTING_UP = 1
    FOLLOWING_FIRST_PASS = 2
    FOLLOWING_BACK = 3
    WAITING_GREEN_BEACON = 4
    CIRCLING_GREEN_BEACON = 5


class FollowBuoyPath(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "follow_path_server", action_name = "follow_buoy_path", search_action_name = "search_followpath")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.declare_parameter("max_gate_pair_dist", 25.0)
        self.max_gate_pair_dist = self.get_parameter("max_gate_pair_dist").get_parameter_value().double_value

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.declare_parameter("safe_margin", 0.2)

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.first_buoy_pair = True

        self.safe_margin = (
            self.get_parameter("safe_margin").get_parameter_value().double_value
        )

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("adapt_dist", 0.7)
        self.adapt_dist = self.get_parameter("adapt_dist").get_parameter_value().double_value

        self.declare_parameter("circle_adapt_dist", 0.3)
        self.circle_adapt_dist = self.get_parameter("circle_adapt_dist").get_parameter_value().double_value

        self.declare_parameter("thresh_dist", 1.5)
        self.thresh_dist = self.get_parameter("thresh_dist").get_parameter_value().double_value

        self.declare_parameter("forward_dist", 5.0)
        self.forward_dist = self.get_parameter("forward_dist").get_parameter_value().double_value

        self.declare_parameter("better_angle_thres", 0.2)
        self.better_angle_thres = self.get_parameter("better_angle_thres").get_parameter_value().double_value

        self.declare_parameter("circle_beacon", True)
        self.circle_beacon = self.get_parameter("circle_beacon").get_parameter_value().bool_value

        self.declare_parameter("beacon_probe_dist", 10.0)
        self.beacon_probe_dist = self.get_parameter("beacon_probe_dist").get_parameter_value().double_value

        self.declare_parameter("green_buoy_loop_count", 2)
        self.green_buoy_loop_count = self.get_parameter("green_buoy_loop_count").get_parameter_value().integer_value

        self.declare_parameter("midpoint_pair_forward_dist", 1.0)
        self.midpoint_pair_forward_dist = self.get_parameter("midpoint_pair_forward_dist").get_parameter_value().double_value

        self.green_labels = set()
        self.red_labels = set()
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
        self.label_mappings = label_mappings
        # hardcoded from reading YAML
        if self.is_sim:
            self.green_labels.add(label_mappings["green"])
            self.red_labels.add(label_mappings["red"])
            self.green_beacon_labels.add(label_mappings["yellow"])
            self.green_beacon_labels.add(label_mappings["blue"])
        else:
            # self.green_labels.add(11) # just to use old rosbags
            # self.red_labels.add(17) # just to use old rosbags
            self.green_labels.add(label_mappings["green_buoy"])
            self.green_labels.add(label_mappings["green_circle"])
            # self.green_labels.add(label_mappings["green_pole_buoy"])
            self.red_labels.add(label_mappings["red_buoy"])
            self.red_labels.add(label_mappings["red_circle"])
            # self.red_labels.add(label_mappings["red_pole_buoy"])
            # self.red_labels.add(label_mappings["yellow_buoy"])
            # self.red_labels.add(label_mappings["yellow_racquet_ball"])
            # self.green_beacon_labels.add(label_mappings["yellow_buoy"])
            # self.green_beacon_labels.add(label_mappings["yellow_racquet_ball"])
            self.green_beacon_labels.add(label_mappings["green_indicator"])
            self.red_beacon_labels.add(label_mappings["red_indicator"])

        self.sent_waypoints = set()

        self.first_setup = True
        self.time_last_seen_buoys = time.time()

        self.obstacles = None

        self.buoy_pairs = []
        self.obstacles = []

        self.sent_forward = False

        self.first_passed_previous = True

        self.state = FollowPathState.SETTING_UP
        self.last_pair = None

        self.pair_to = None

        self.first_back = True

        self.tracked_buoys = []
        self.max_tracked_buoy_id = -1

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

    def next_pair(self, prev_pair, red, green):
        """
        Returns the next buoy pair from the previous pair,
        by checking the closest one to the middle of the previous buoy pair that's in front of the pair
        """
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
            return None

        if prev_pair is not None:
            prev_pair_midpoint = midpoint_pair_dir(prev_pair, 0.0)[0]
            self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
            # Add a threshold on the minimum distance to a buoy if the new angle on the not close buoys is worse (diagonal but not the one that improves the pair)
            # Instead of only checking the two closest, order by distance and pick either the first one not at duplicate distance or the furthest one
            # If duplicate distance for both reject, if one is duplicate distance check angles
            left_duplicate, left_next = pick_buoy(front_red if self.red_left else front_green, prev_pair_midpoint, prev_pair.left, self.duplicate_dist)
            right_duplicate, right_next = pick_buoy(front_green if self.red_left else front_red, prev_pair_midpoint, prev_pair.right, self.duplicate_dist)

            ret = None

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

                    if (ret is None) or better_buoy_pair_transition(ret, cur, prev_pair, self.buoy_pair_dist_thres, self.better_angle_thres, self.duplicate_dist):
                        ret = cur
            return ret
        else:
            self.get_logger().debug("No previous pair!")
            # TODO: change those to: get_closest_to((0,0), red/green, local=True) to not have to use odometry position but just local obstacle positions (wrt the robot)?
            self.get_logger().debug(
                f"next buoys: red: {get_closest_to(self.robot_pos, red)}, green: {get_closest_to(self.robot_pos, green)}"
            )
            # if there is no previous pair, just take the closest red and green buoys
            if self.red_left:
                return InternalBuoyPair(
                    get_closest_to(self.robot_pos, red),
                    get_closest_to(self.robot_pos, green),
                )
            else:
                return InternalBuoyPair(
                    get_closest_to(self.robot_pos, green),
                    get_closest_to(self.robot_pos, red),
                )

    def find_better_pair_to(self, curr_pair, left_buoys, right_buoys):
        changed = False
        new_right = curr_pair.right
        origin = np.array([0.0, 0.0])
        for buoy in right_buoys:
            if np.array_equal(ob_coords(buoy), ob_coords(curr_pair.right)):
                continue
            if np.linalg.norm(ob_coords(curr_pair.left) - ob_coords(buoy)) < self.inter_buoy_pair_dist:
                continue
            if not ccw(origin, ob_coords(buoy, local=True), ob_coords(curr_pair.left, local=True)):
                continue
            if check_better_one_side(curr_pair.left, curr_pair.right, buoy, self.better_angle_thres):
                new_right = buoy
                changed = True
        curr_pair.right = new_right
        new_left = curr_pair.left
        for buoy in left_buoys:
            if np.array_equal(ob_coords(buoy), ob_coords(curr_pair.left)):
                continue
            if np.linalg.norm(ob_coords(curr_pair.right) - ob_coords(buoy)) < self.inter_buoy_pair_dist:
                continue
            if not ccw(origin, ob_coords(curr_pair.right, local=True), ob_coords(buoy, local=True)):
                continue
            if check_better_one_side(curr_pair.right, curr_pair.left, buoy, self.better_angle_thres):
                new_left = buoy
                changed = True
        curr_pair.left = new_left
        return changed

    def generate_waypoints(self):
        """
        Runs every time a new obstacle map is received, keeps
        track of the pair of buoys the robot is heading towards,
        checks if it passed it (the robot is in front of the pair of buoys)
        (#TODO: add a margin of error such that the robot is considered to have passed the buoys if it's
        a bit in front of them) and update the pair accordingly, and afterwards computes
        the sequence of future waypoints based on the first waypoint
        and the next_pair() function to compute the next pair from each one in the sequence,
        as long as there is a next pair from the buoys that are stored in the obstacle map.
        """
        # split the buoys into red and green
        green_buoys, red_buoys = split_buoys(self.obstacles, self.green_labels, self.red_labels)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, red buoys: {obs_to_pos(red_buoys)}, green buoys: {obs_to_pos(green_buoys)}"
        )

        if self.gate_pair is not None:
            self.pair_to = self.gate_pair

        if self.pair_to is None:
            self.get_logger().debug("No pair to go to.")
            self.buoy_pairs = []
            self.pair_to = None
            self.first_buoy_pair = True
            if not self.setup_buoys(visualize=False):
                return

        """
        Update the current sequence if better transitions are found
        """
        # Update first pair (if it exists) in case the current one is worse
        # an interesting criterion that could work well is if the triangle with the old red, old green, and new green, has almost a right angle at the new green, or just a larger acute angle than the old green one
        # also do that for new pair transitions, checking for the new angles in both colors when considering a new pair -> maybe switch if either angle is better -> still keep the distance constraint
        changed_pair_to = False
        adapt_waypoint = False
        if len(self.buoy_pairs) != 0:
            self.buoy_pairs[0].left, res_left_left = replace_closest(self.buoy_pairs[0].left, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
            self.buoy_pairs[0].right, res_right_right = replace_closest(self.buoy_pairs[0].right, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
            _, res_left_right = replace_closest(self.buoy_pairs[0].left, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
            _, res_right_left = replace_closest(self.buoy_pairs[0].right, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
            # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint, it is false
            if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                self.buoy_pairs = []
                self.pair_to = None
                self.first_buoy_pair = True
                self.get_logger().info('WE ARE GOING TO A FAKE PAIR, FIND PATH AGAIN')
                if not self.setup_buoys(visualize=False):
                    return
            else:
                # Check if new target waypoint is further than adapt_dist away from the old one that's been sent (store it in a global variable and only change it when sending to server)
                if (self.sent_waypoint is not None) and (np.linalg.norm(midpoint_pair_dir(self.buoy_pairs[0], self.midpoint_pair_forward_dist)[0] - np.array(self.sent_waypoint)) > self.adapt_dist):
                    adapt_waypoint = True
                # changed_pair_to = self.find_better_pair_to(self.buoy_pairs[0], red_buoys if self.red_left else green_buoys, green_buoys if self.red_left else red_buoys)
                self.pair_to = self.buoy_pairs[0]
        ind = 0
        while ind < len(self.buoy_pairs):
            # Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair
            if ind != 0:
                self.buoy_pairs[ind].left, res_left_left = replace_closest(self.buoy_pairs[ind].left, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
                self.buoy_pairs[ind].right, res_right_right = replace_closest(self.buoy_pairs[ind].right, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
                _, res_left_right = replace_closest(self.buoy_pairs[ind].left, green_buoys if self.red_left else red_buoys, self.duplicate_dist)
                _, res_right_left = replace_closest(self.buoy_pairs[ind].right, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
                # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint
                if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                    self.buoy_pairs = self.buoy_pairs[:ind]
                    break
            # Find potential better next pair
            next_pair = self.next_pair(self.buoy_pairs[ind], red_buoys, green_buoys)
            if next_pair is not None and ((ind == (len(self.buoy_pairs)-1) and buoy_pairs_distance(self.buoy_pairs[ind], next_pair, "mid") > self.buoy_pair_dist_thres) or (ind < (len(self.buoy_pairs)-1) and better_buoy_pair_transition(self.buoy_pairs[ind+1],next_pair,self.buoy_pairs[ind], self.buoy_pair_dist_thres, self.better_angle_thres, self.duplicate_dist))):
                self.buoy_pairs = self.buoy_pairs[:ind+1]
                self.buoy_pairs.append(next_pair)
            ind += 1

        passed_previous = False
        # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
        self.last_pair = self.pair_to
        left_coords = ob_coords(self.pair_to.left)
        right_coords = ob_coords(self.pair_to.right)
        wpt_pos = midpoint_pair_dir(self.pair_to, self.midpoint_pair_forward_dist)[0]
        if ccw(
            left_coords,
            right_coords,
            self.robot_pos,
        ) or (np.sum((wpt_pos - self.robot_pos)**2) <= self.thresh_dist ** 2) or self.moved_to_point:
            passed_previous = True

        if self.first_buoy_pair:
            self.buoy_pairs = [self.pair_to]
            self.time_last_seen_buoys = time.time()
        elif passed_previous:
            if len(self.buoy_pairs)>=2:
                self.buoy_pairs = self.buoy_pairs[1:]
                self.time_last_seen_buoys = time.time()
                self.pair_to = self.buoy_pairs[0]
                self.sent_forward = False
            elif len(self.buoy_pairs) == 1:
                # TODO: Add a check for whether we searched for buoys by going right and left, and search if not, return, then if we already searched move on
                if not self.sent_forward:
                    buoy_pair = self.buoy_pairs[0]
                    left_coords = ob_coords(buoy_pair.left)
                    right_coords = ob_coords(buoy_pair.right)
                    wpt = midpoint_pair_dir(buoy_pair, self.forward_dist)[0]

                    self.send_waypoint_to_server(wpt)

                    self.get_logger().info("FORWARD WAYPOINT")

                    self.sent_forward = True

                    return
                else:
                    if self.moved_to_point:
                        if self.state == FollowPathState.FOLLOWING_FIRST_PASS:
                            self.state = FollowPathState.WAITING_GREEN_BEACON if self.circle_beacon else FollowPathState.FOLLOWING_BACK
                            self.pair_to = self.last_pair
                            self.first_buoy_pair = True
                            self.red_left = not self.red_left
                            self.pair_to.left, self.pair_to.right = self.pair_to.right, self.pair_to.left
                            return
                        else:
                            self.mark_successful()
                            return
                    else:
                        return
            else:
                # TODO: Add a check for whether we searched for buoys by going right and left, and search if not, return, then if we already searched move on
                if time.time() - self.time_last_seen_buoys > 5:
                    if self.state == FollowPathState.FOLLOWING_FIRST_PASS:
                        self.state = FollowPathState.WAITING_GREEN_BEACON if self.circle_beacon else FollowPathState.FOLLOWING_BACK
                        self.pair_to = self.last_pair
                        self.first_buoy_pair = True
                        self.red_left = not self.red_left
                        self.pair_to.left, self.pair_to.right = self.pair_to.right, self.pair_to.left
                        return
                    else:
                        self.mark_successful()
                        return
        else:
            self.sent_forward = False
            self.time_last_seen_buoys = time.time()

        self.waypoints = [midpoint_pair_dir(pair, self.midpoint_pair_forward_dist)[0] for pair in self.buoy_pairs]

        self.get_logger().debug(f"Pairs: {[(ob_coords(pair.left), ob_coords(pair.right)) for pair in self.buoy_pairs]}")
        self.get_logger().debug(f"Waypoints: {self.waypoints}")

        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        self.waypoint_marker_pub.publish(buoy_pairs_to_markers([(pair.left, pair.right, pair_angle_to_pose(
            pair=wpt,
            # angle=(
            #     math.atan(ob_coords(pair.right)[1] - ob_coords(pair.left)[1]) /
            #     (ob_coords(pair.right)[0] - ob_coords(pair.left)[0])
            # ) + (math.pi / 2),
            angle=0,
        ), np.linalg.norm(ob_coords(pair.left) - ob_coords(pair.right))/2 - self.safe_margin) for wpt, pair in zip(self.waypoints, self.buoy_pairs)], self.red_left, self.global_frame_id))

        if self.waypoints:
            waypoint = self.waypoints[0]
            self.get_logger().debug(
                f"cur_waypoint: {waypoint}, sent_waypoints: {self.sent_waypoints}"
            )
            self.get_logger().debug(f"len(waypoints): {len(self.waypoints)}")

            # if not passed_waypoint:
            if (passed_previous and self.first_passed_previous) or self.first_buoy_pair or changed_pair_to or (adapt_waypoint and (not passed_previous)):
                self.get_logger().info('SENDING WAYPOINT')
                # self.get_logger().info(f'passed: {passed_previous}, first passed: {passed_previous}, first buoy pair: {self.first_buoy_pair}, changed pair to: {changed_pair_to}, adapt waypoint: {adapt_waypoint}')
                self.send_waypoint_to_server(waypoint)
                self.sent_waypoints.add(tuple(waypoint))
                self.first_buoy_pair = False
            elif self.sent_waypoint is not None and (self.waypoint_rejected or self.waypoint_aborted):
                    self.get_logger().info("Waypoint request aborted by nav server and no new waypoint option found. Resending request...")
                    self.send_waypoint_to_server(self.sent_waypoint)
                    # Waypoint has already been sent before, should be fine to avoid adding it to set?
            if passed_previous:
                self.first_passed_previous = False
        if not passed_previous:
            self.first_passed_previous = True

    def adapt_pair_to(self):
        green_buoys, red_buoys = split_buoys(self.obstacles, self.green_labels, self.red_labels)
        self.pair_to.left, _ = replace_closest(self.pair_to.left, red_buoys if self.red_left else green_buoys, self.duplicate_dist)
        self.pair_to.right, _ = replace_closest(self.pair_to.right, green_buoys if self.red_left else red_buoys, self.duplicate_dist)

    def report_new_obstacles(self):
        obstacle: Obstacle
        for obstacle in self.obstacles:
            # TODO discard objects like dock or miscellaneous that shouldn't be tracked/don't exist in the course
            if obstacle.id > self.max_tracked_buoy_id:
                # potentially new obstacle
                new_obs = True
                tracked_obs: Obstacle
                for tracked_obs in self.tracked_buoys:
                    if np.linalg.norm(ob_coords(tracked_obs) - ob_coords(obstacle)) < self.duplicate_dist:
                        new_obs = False
                        break
                if new_obs:
                    # report & add to tracked obs
                    self.tracked_buoys.append(obstacle)
                    obs_type = ObjectType.OBJECT_UNKNOWN
                    obs_color = Color.COLOR_UNKNOWN
                    if obstacle.label in self.green_beacon_labels:
                        obs_type = ObjectType.OBJECT_LIGHT_BEACON
                        obs_color = Color.COLOR_GREEN
                    elif obstacle.label in self.red_beacon_labels:
                        obs_type = ObjectType.OBJECT_LIGHT_BEACON
                        obs_color = Color.COLOR_RED
                    elif obstacle.label in self.green_labels:
                        obs_type = ObjectType.OBJECT_BUOY
                        obs_color = Color.COLOR_GREEN
                    elif obstacle.label in self.red_labels:
                        obs_type = ObjectType.OBJECT_BUOY
                        obs_color = Color.COLOR_RED
                    else:
                        obs_type = ObjectType.OBJECT_BUOY
                        obs_color = Color.COLOR_BLACK
                    self.report_data(ObjectDetected(
                        object_type=obs_type,
                        color=obs_color,
                        position=self.pos_to_latlng(self.latlng_origin, ob_coords(obstacle)),
                        object_id=obstacle.id,
                        task_context=TaskType.TASK_NAV_CHANNEL))


    def map_cb(self, msg):
        """
        When a new map is received, check if it is the first one (we haven't set up the starting buoys)
        and find the starting pair, and then (if the starting buoys are successfully computed) form
        the buoy pair / waypoint sequence
        """
        self.obstacles = msg.obstacles
        if self.paused:
            return
        if self.state in [FollowPathState.WAITING_GREEN_BEACON, FollowPathState.CIRCLING_GREEN_BEACON]:
            self.adapt_pair_to()
        self.report_new_obstacles()

    def return_to_start(self):
        '''
        After circling the buoy, return to the starting position.
        '''
        self.get_logger().info("Returning to start")
        self.move_to_point(self.home_pos, busy_wait=True)
        return Task.Result(success=True)


    def station_hold(self):
        nav_x, nav_y, heading = self.get_robot_pose()

        self.get_logger().info(f"Station hold at ({nav_x:.2f}, {nav_y:.2f}, {heading:.1f})")

        # LOOK LEFT 30 DEG

        self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, cancel_on_exit=True)

        # LOOK RIGHT 30 DEG

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, cancel_on_exit=True)

        # BACK TO FORWARD

        self.move_to_waypoint([nav_x, nav_y, heading], is_stationary=True, busy_wait=False, cancel_on_exit=True)

    def search_buoys(self):
        nav_x, nav_y, heading = self.get_robot_pose()

        # LOOK LEFT 30 DEG

        self.get_logger().info(f"Turning left")

        self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, cancel_on_exit=True)

        # LOOK RIGHT 30 DEG

        self.get_logger().info(f"Turning right")

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, cancel_on_exit=True)

    def search_beacon(self):
        nav_x, nav_y, heading = self.get_robot_pose()

        # # LOOK LEFT 30 DEG

        self.get_logger().info(f"Turning left")

        self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, exit_func=partial(self.buoy_detected, self.obstacles, self.green_beacon_labels, self.duplicate_dist), cancel_on_exit=True)

        # LOOK RIGHT 30 DEG

        self.get_logger().info(f"Turning right")

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, exit_func=partial(self.buoy_detected, self.obstacles, self.green_beacon_labels, self.duplicate_dist), cancel_on_exit=True)

    def should_accept_task(self, goal_request):
        if self.first_run:
            if self.obstacles is None:
                return False
            self.first_setup = True
            return self.setup_buoys(visualize=False)
        else:
            return True

    # def init_setup(self):
    #     if self.obstacles is None:
    #         return
    #     success = self.setup_buoys(visualize=False)
    #     if success:
    #         self.get_logger().info("Setup buoys succeeded!")
    #         self.state = FollowPathState.FOLLOWING_FIRST_PASS
    #         self.mark_successful()

    def init_setup(self):
        if self.first_run:
            self.get_logger().info("Setup buoys succeeded!")
            self.state = FollowPathState.FOLLOWING_FIRST_PASS
            self.mark_successful()
        else:
            self.get_logger().info("Restarting...")
            self.first_buoy_pair = True
            self.mark_successful()

    def control_loop(self):
        # self.station_hold()
        if self.state in [FollowPathState.FOLLOWING_FIRST_PASS, FollowPathState.FOLLOWING_BACK]:
            if self.state == FollowPathState.FOLLOWING_BACK and self.first_back:
                if "green_pole_buoy" in self.green_labels:
                    self.green_labels.remove(self.label_mappings["green_pole_buoy"])
                if "red_pole_buoy" in self.red_labels:
                    self.red_labels.remove(self.label_mappings["red_pole_buoy"])
                # make the robot face the previous gate
                # self.get_logger().info('facing gate')
                # _, intended_dir = midpoint_pair_dir(self.pair_to, 0.0)
                # theta_intended = math.atan2(intended_dir[1], intended_dir[0])
                # nav_x, nav_y = self.robot_pos
                # self.move_to_waypoint([nav_x, nav_y, theta_intended], is_stationary=False, busy_wait=True, cancel_on_exit=True)
                # recompute gate
                # self.setup_buoys(visualize=False)
                gate_mid, _ = midpoint_pair_dir(self.pair_to, 0.0)
                self.setup_buoys(gate_mid - self.robot_pos, visualize=False)
                self.get_logger().info('recomputing gate')
                self.first_back = False
            self.generate_waypoints()
        elif self.state == FollowPathState.WAITING_GREEN_BEACON:
            self.get_logger().info(f"Searching green beacon")

            # self.search_beacon()

            self.get_logger().info(f'Detecting green beacon')
            # self.home_pos = self.robot_pos # keep track of home position
            # self.buoy_direction = self.robot_dir
            self.home_pos, self.buoy_direction = midpoint_pair_dir(self.pair_to, 0.0)
            self.get_logger().info(f"Facing direction: {self.buoy_direction}")
            action_result = self.probe_buoy(self.buoy_direction, self.beacon_probe_dist, partial(self.buoy_detected, self.obstacles, self.green_beacon_labels, self.duplicate_dist))
            if action_result.success == False:
                self.state = FollowPathState.FOLLOWING_BACK
            else:
                self.state = FollowPathState.CIRCLING_GREEN_BEACON
        elif self.state == FollowPathState.CIRCLING_GREEN_BEACON:
            self.get_logger().info(f'Circling green beacon')
            self.gate_wpt, _ = midpoint_pair_dir(self.pair_to, 0.0)
            action_result = self.smooth_circle_buoy(self.green_beacon_labels, self.circle_adapt_dist, self.duplicate_dist, self.gate_wpt, self.green_buoy_loop_count)
            # action_result = self.return_to_start()
            self.state = FollowPathState.FOLLOWING_BACK

def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPath()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
