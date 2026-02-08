#!/usr/bin/env python3
from ast import Num
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

import os
import yaml
import math
import time
from collections import deque
from functools import partial
from enum import Enum

TIMER_PERIOD = 1 / 60

class ReturnState(Enum):
    SETTING_UP = 1
    RETURNING = 2

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

class ReturnHome(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "return_home_server", action_name = "return_home", search_action_name = "search_return")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter("red_left", False)
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
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(gate_wpt), 0.0)]))
        return gate_wpt
    
    def should_accept_task(self, goal_request):
        if self.obstacles is None:
            return False
        return self.setup_buoys()

    def init_setup(self):
        self.get_logger().info("Setup buoys succeeded!")
        self.state = ReturnState.RETURNING
        self.mark_successful()

    def control_loop(self):
        self.return_to_start()
        self.mark_successful()
        

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles
    
    def return_to_start(self):
        self.get_logger().info("Returning to home")
        self.new_gate_pair = self.gate_pair
        while self.new_gate_pair is not None:
            self.gate_pair = self.new_gate_pair
            self.gate_wpt, _ = self.midpoint_pair_dir(self.gate_pair, -self.forward_dist_back)
            self.move_to_point(self.gate_wpt, busy_wait=True,
                goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, -self.forward_dist_back)))
            self.new_gate_pair = None
            self.gate_wpt, _ = self.midpoint_pair_dir(self.gate_pair, self.gate_probe_dist)
            self.move_to_point(self.gate_wpt, busy_wait=True,
                exit_func=partial(self.next_pair, self.gate_pair), goal_update_func=partial(self.update_point, "gate_wpt", self.adaptive_distance, partial(self.update_gate_wpt_pos, self.gate_probe_dist)))
        
        self.report_data(GatePass(
            type=GateType.GATE_EXIT,
            position=self.pos_to_latlng(self.latlng_origin, self.robot_pos)))

        return Task.Result(success=True)

    def norm_squared(self, vec, ref=(0, 0)):
        return (vec[0] - ref[0])**2 + (vec[1]-ref[1])**2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))
    
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

    
    def setup_buoys(self, ref_pair = None):
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
            if self.norm(self.robot_pos, self.ob_coords(ob)) < self.gate_dist_thres        
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
        green_to = None
        red_to = None
        for red_b in red_buoys:
            for green_b in green_buoys:
                if self.norm(self.ob_coords(red_b), self.ob_coords(green_b)) < self.inter_buoy_pair_dist or self.norm(self.ob_coords(red_b), self.ob_coords(green_b)) > self.max_inter_gate_dist:
                    continue
                # elif ((green_to is None) or (self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True))) < self.norm(self.midpoint(self.ob_coords(red_to, local=True), self.ob_coords(green_to, local=True))))) and (self.red_left == self.ccw((0, 0), self.ob_coords(green_b, local=True), self.ob_coords(red_b, local=True))):
                elif ((green_to is None) or (self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True))) < self.norm(self.midpoint(self.ob_coords(red_to, local=True), self.ob_coords(green_to, local=True))))):
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
        self.gate_mid, self.gate_dir = self.midpoint_pair_dir(self.gate_pair, 0.0)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(self.gate_mid), 0.0)]))
        return True
    
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
    
    def buoy_pairs_distance(self, p1, p2, mode="min", loc=False):
        p1_left, p1_right = self.ob_coords(p1.left, local=loc), self.ob_coords(p1.right, local=loc)
        p2_left, p2_right = self.ob_coords(p2.left, local=loc), self.ob_coords(p2.right, local=loc)
        if mode=="mid":
            p1_mid = ((p1_right[0]+p1_left[0])/2.0, (p1_right[1]+p1_left[1])/2.0)
            p2_mid = ((p2_right[0]+p2_left[0])/2.0, (p2_right[1]+p2_left[1])/2.0)
            dist = self.norm(p1_mid, p2_mid)
        else:
            left_diff = (p2_left[0]-p1_left[0], p2_left[1]-p1_left[1])
            right_diff = (p2_right[0]-p1_right[0], p2_right[1]-p1_right[1])
            dist = min(self.norm(left_diff), self.norm(right_diff))
        return dist
    
    def buoy_pairs_angle(self, p1, p2, loc=False):
        p1_left, p1_right = (self.ob_coords(p1.left, local=loc), self.ob_coords(p1.right, local=loc))
        p2_left, p2_right = (self.ob_coords(p2.left, local=loc), self.ob_coords(p2.right, local=loc))
        p1_diff = (p1_right[0]-p1_left[0], p1_right[1]-p1_left[1])
        p2_diff = (p2_right[0]-p2_left[0], p2_right[1]-p2_left[1])
        # TODO: change angle to be away from the boat, use dot product
        angle = math.acos((p1_diff[0]*p2_diff[0]+p1_diff[1]*p2_diff[1])/(self.norm(p1_diff)*self.norm(p2_diff)))
        return angle
    
    def get_acute_angle(self, angle):
        ret_angle = angle
        if ret_angle < 0:
            ret_angle = -ret_angle
        if ret_angle > math.pi/2.0:
            ret_angle = math.pi - ret_angle
        return ret_angle

    
    def better_buoy_pair_transition(self, p_old, p_new, p_ref, mode="both"): # mode can be both or either
        # return ((self.buoy_pairs_distance(p_ref, p_new) <= self.buoy_pair_dist_thres and 
        #         self.buoy_pairs_distance(p_ref, p_new, mode="min") > self.buoy_pairs_distance(p_ref, p_old, mode="min")) or
        # return (self.buoy_pairs_distance(p_ref, p_new) > self.buoy_pair_dist_thres and
        #         (self.buoy_pairs_distance(p_ref, p_old) <= self.buoy_pair_dist_thres or
        #          self.buoy_pairs_angle(p_ref, p_old) > self.buoy_pairs_angle(p_ref, p_new)))
        # want new pair to have midpoint distance from previous pair larger than the threshold
        # above that threshold we check if either/both angles are better (closer to right angle)
        # TODO: Add a max distance threshold (although will probably not be an issue since we only see the next couple pairs at most)
        return (self.buoy_pairs_distance(p_ref, p_new, "mid") > self.buoy_pair_dist_thres and
                (self.buoy_pairs_distance(p_ref, p_old, "mid") <= self.buoy_pair_dist_thres or
                 self.check_better_pair_angles(p_ref, p_old, p_new, mode)))
                 
        
    def check_better_pair_angles(self, ref_pair, old_pair, new_pair, mode="both"): # mode can be both or either
        """
        Checks if a potential next pair is better than the current next pair, wrt to the old one
        A
        |\ 
        | \ 
        C----D
        |   \|
        |    B
        |    |
        E----F
        CD is better than AB wrt EF (closer to right angles wrt to E, F)
        """
        old_left_duplicate = (self.norm(self.ob_coords(ref_pair.left), self.ob_coords(old_pair.left)) < self.duplicate_dist)
        old_right_duplicate = (self.norm(self.ob_coords(ref_pair.right), self.ob_coords(old_pair.right)) < self.duplicate_dist)
        new_left_duplicate = (self.norm(self.ob_coords(ref_pair.left), self.ob_coords(new_pair.left)) < self.duplicate_dist)
        new_right_duplicate = (self.norm(self.ob_coords(ref_pair.right), self.ob_coords(new_pair.right)) < self.duplicate_dist)

        old_left = 0 if old_left_duplicate else self.get_triangle_angle(ref_pair.left, old_pair.left, old_pair.right)
        old_right = 0 if old_right_duplicate else self.get_triangle_angle(ref_pair.right, old_pair.right, old_pair.left)
        new_left = 0 if new_left_duplicate else self.get_triangle_angle(ref_pair.left, new_pair.left, new_pair.right)
        new_right = 0 if new_right_duplicate else self.get_triangle_angle(ref_pair.right, new_pair.right, new_pair.left)

        # old and new can't be duplicate in both, because they would not be considered as potential next pairs
        if ((old_left_duplicate or old_right_duplicate) and (new_left_duplicate or new_right_duplicate)):
            old_angle = old_left if old_right_duplicate else old_right
            new_angle = new_left if new_right_duplicate else new_right
            return new_angle > old_angle + self.better_angle_thres
        elif ((old_left_duplicate or old_right_duplicate) and (new_left_duplicate or new_right_duplicate)):
            return (old_left_duplicate or old_right_duplicate)
        else:
            return ((new_left > (old_left + self.better_angle_thres)) and (new_right > (old_right + self.better_angle_thres))) if (mode == "both") else (new_left > (old_left + self.better_angle_thres)) or (new_right > (old_right + self.better_angle_thres))
        
    def get_triangle_angle(self, buoy_a, buoy_b, buoy_c):
        return self.get_acute_angle(self.buoy_pairs_angle(InternalBuoyPair(buoy_a, buoy_b), InternalBuoyPair(buoy_b, buoy_c)))
    
    def check_better_one_side(self, ref_buoy, old_buoy, new_buoy):
        """
        Returns whether the new buoy is closer to right angle compared to the old one, wrt to the reference buoy (and the other buoy as part of the path)
        A
        |\
        | \
        |  \
        B---C
        B is better than C (reference buoy is A)
        """
        old_angle = self.get_triangle_angle(ref_buoy, old_buoy, new_buoy)
        new_angle = self.get_triangle_angle(ref_buoy, new_buoy, old_buoy)
        return new_angle > (old_angle + self.better_angle_thres)

    def next_pair(self, prev_pair):
        """
        Returns the next buoy pair from the previous pair,
        by checking the closest one to the middle of the previous buoy pair that's in front of the pair
        """
        green, red = self.split_buoys(self.obstacles)
        front_red = self.filter_front_buoys(prev_pair, red)
        front_green = self.filter_front_buoys(prev_pair, green)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, front red buoys: {self.obs_to_pos(front_red)}, front green buoys: {self.obs_to_pos(front_green)}"
        )

        if ((not front_red) and (not front_green)) or ((prev_pair is None) and ((not front_red) or (not front_green))):
            prev_coords = self.ob_coords(prev_pair.left), self.ob_coords(
                prev_pair.right
            )
            red_coords = self.obs_to_pos(red)
            green_coords = self.obs_to_pos(green)

            self.get_logger().debug(
                f"buoys:  {prev_coords} \nred: {red_coords} \ngreen: {green_coords}"
            )
            self.get_logger().debug(f"robot: {self.robot_pos}")
            self.get_logger().debug("Missing at least one front buoy")
            return False

        prev_pair_midpoint, _ = self.midpoint_pair_dir(prev_pair, 0.0)
        self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
        # Add a threshold on the minimum distance to a buoy if the new angle on the not close buoys is worse (diagonal but not the one that improves the pair)
        # Instead of only checking the two closest, order by distance and pick either the first one not at duplicate distance or the furthest one
        # If duplicate distance for both reject, if one is duplicate distance check angles
        left_duplicate, left_next = self.pick_buoy(front_red if self.red_left else front_green, prev_pair_midpoint, prev_pair.left)
        right_duplicate, right_next = self.pick_buoy(front_green if self.red_left else front_red, prev_pair_midpoint, prev_pair.right)

        improved = False        
        for left_buoy in ((True, prev_pair.left), (left_duplicate, left_next)):
            for right_buoy in ((True, prev_pair.right), (right_duplicate, right_next)):
                if left_buoy[0] and right_buoy[0]:
                    continue
                
                if left_buoy[0] and (not self.check_better_one_side(prev_pair.left, prev_pair.right, right_buoy[1])):
                    continue
                
                if right_buoy[0] and (not self.check_better_one_side(prev_pair.right, prev_pair.left, left_buoy[1])):
                    continue

                if self.norm(self.ob_coords(left_buoy[1]), self.ob_coords(right_buoy[1])) < self.inter_buoy_pair_dist or self.norm(self.ob_coords(left_buoy[1]), self.ob_coords(right_buoy[1])) > self.max_inter_gate_dist:
                    continue

                cur = InternalBuoyPair(left_buoy[1], right_buoy[1]) 

                if self.buoy_pairs_distance(prev_pair, cur, "mid") > self.max_gate_pair_dist:
                    continue
                
                if (self.new_gate_pair is None) or self.better_buoy_pair_transition(self.new_gate_pair, cur, prev_pair):
                    self.new_gate_pair = cur
                    improved = True
        return improved

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


def main(args=None):
    rclpy.init(args=args)
    node = ReturnHome()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
