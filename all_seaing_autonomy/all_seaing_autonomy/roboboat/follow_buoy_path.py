#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import FollowPath, Task, Waypoint
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from action_msgs.msg import GoalStatus

import math
import os
import yaml
import time
from enum import Enum
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

class FollowPathState(Enum):
    SETTING_UP = 1
    FOLLOWING_FIRST_PASS = 2
    FOLLOWING_BACK = 3
    WAITING_GREEN_BEACON = 4
    CIRCLING_GREEN_BEACON = 5


class FollowBuoyPath(ActionServerBase):
    def __init__(self):
        super().__init__("follow_path_server")

        self._action_server = ActionServer(
            self,
            Task,
            "follow_buoy_path",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/labeled", self.map_cb, 10
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")

        self.declare_parameter("xy_threshold", 1.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("wpt_theta_threshold", 10.0)
        self.declare_parameter("goal_tol", 1.0)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 10)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")
        self.declare_parameter("bypass_planner", False)

        self.bypass_planner = self.get_parameter("bypass_planner").get_parameter_value().bool_value

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.declare_parameter("inter_buoy_pair_dist", 1.0)
        self.inter_buoy_pair_dist = self.get_parameter("inter_buoy_pair_dist").get_parameter_value().double_value

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

        self.declare_parameter("thresh_dist", 0.5)
        self.thresh_dist = self.get_parameter("thresh_dist").get_parameter_value().double_value

        self.declare_parameter("forward_dist", 5.0)
        self.forward_dist = self.get_parameter("forward_dist").get_parameter_value().double_value

        self.declare_parameter("better_angle_thres", 0.2)
        self.better_angle_thres = self.get_parameter("better_angle_thres").get_parameter_value().double_value

        self.declare_parameter("timer_period", 1/30.0)
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        self.declare_parameter("circle_beacon", True)
        self.circle_beacon = self.get_parameter("circle_beacon").get_parameter_value().bool_value

        self.declare_parameter("beacon_probe_dist", 10.0)
        self.beacon_probe_dist = self.get_parameter("beacon_probe_dist").get_parameter_value().double_value

        self.declare_parameter("turn_offset", 5.0)
        self.turn_offset = self.get_parameter("turn_offset").get_parameter_value().double_value
        
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
            self.green_labels.add(label_mappings["green_pole_buoy"])
            self.red_labels.add(label_mappings["red_buoy"])
            self.red_labels.add(label_mappings["red_circle"])
            self.red_labels.add(label_mappings["red_pole_buoy"])
            self.red_labels.add(label_mappings["red_racquet_ball"])
            self.green_beacon_labels.add(label_mappings["yellow_buoy"])
            self.green_beacon_labels.add(label_mappings["yellow_racquet_ball"])
        
        self.sent_waypoints = set()

        self.red_left = True
        self.first_setup = True
        self.result = False
        self.time_last_seen_buoys = time.time()

        self.obstacles = None

        self.buoy_pairs = []
        self.obstacles = []

        self.sent_forward = False

        self.sent_waypoint = None

        self.first_passed_previous = True

        self.lastSelectedGoal = None
        self.waypoint_sent_future = None
        self.send_goal_future = None

        self.state = FollowPathState.SETTING_UP
        self.last_pair = None

        self.green_beacon_found = False
        self.waypoint_reject = True

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

    def midpoint_pair(self, pair):
        left_coords = self.ob_coords(pair.left)
        right_coords = self.ob_coords(pair.right)
        midpoint = self.midpoint(left_coords, right_coords)
        
        scale = 1 # number of meters to translate forward. TODO: parametrize.
        dy = right_coords[1] - left_coords[1]
        dx = right_coords[0] - left_coords[0]
        norm = math.sqrt(dx**2 + dy**2)
        dx /= norm
        dy /= norm
        midpoint = (midpoint[0] - scale*dy, midpoint[1] + scale*dx)

        return midpoint

    @property
    def robot_pos(self):
        '''
        Gets the robot position as a tuple (x,y)
        '''
        position = self.get_robot_pose()[0:2]
        return (float(position[0]), float(position[1]))

    @property
    def robot_dir(self):
        '''
        Gets the robot direction as a tuple, containing the unit vector in the same direction as heading
        '''
        heading = self.get_robot_pose()[2]
        return (math.cos(heading), math.sin(heading))

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
            # marker_array.markers.append(
            #     Marker(
            #         type=Marker.CYLINDER,
            #         pose=Pose(
            #             position=Point(
            #                 x=point.position.x,
            #                 y=point.position.y,
            #             )
            #         ),
            #         header=Header(frame_id=self.global_frame_id),
            #         scale=Vector3(
            #             x=radius, y=radius, z=1.0
            #         ),
            #         color=ColorRGBA(g=1.0, a=0.5),
            #         id=(4 * i) + 3,
            #     )
            # )
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
        # closest_red = self.get_closest_to((0, 0), red_buoys, local=True)
        # closest_green = self.get_closest_to((0, 0), green_buoys, local=True)
        # if self.ccw((0, 0), self.ob_coords(closest_green, local=True), self.ob_coords(closest_red, local=True)):
        #     self.red_left = True
        #     self.pair_to = InternalBuoyPair(closest_red, closest_green)
        #     self.get_logger().info("RED BUOYS LEFT, GREEN BUOYS RIGHT")
        # else:
        #     self.red_left = False
        #     self.pair_to = InternalBuoyPair(closest_green, closest_red)
        #     self.get_logger().info("GREEN BUOYS LEFT, RED BUOYS RIGHT")
        # self.backup_pair = None
        # want to pick the pair that's far apart but has the closest midpoint
        # TODO: Add more conditions on judging what the best pair is, other than just distance (e.g. should not be really angled wrt the boat etc.)
        if self.first_setup:
            green_to = None
            red_to = None
            for red_b in red_buoys:
                for green_b in green_buoys:
                    if self.norm(self.ob_coords(red_b), self.ob_coords(green_b)) < self.inter_buoy_pair_dist:
                        self.get_logger().debug(f'RED: {self.ob_coords(red_b)}, GREEN: {self.ob_coords(green_b)} REJECTED, INTER-BUOY DIST: {self.norm(self.ob_coords(red_b), self.ob_coords(green_b))} < {self.inter_buoy_pair_dist}')
                        continue
                    elif (green_to is None) or (self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True))) < self.norm(self.midpoint(self.ob_coords(red_to, local=True), self.ob_coords(green_to, local=True)))):
                        self.get_logger().debug(f'RED: {self.ob_coords(red_b)}, GREEN: {self.ob_coords(green_b)} BETTER, DIST FROM ROBOT: {self.norm(self.midpoint(self.ob_coords(red_b, local=True), self.ob_coords(green_b, local=True)))}')
                        green_to = green_b
                        red_to = red_b
            if green_to is None:
                return False
            if self.ccw((0, 0), self.ob_coords(green_to, local=True), self.ob_coords(red_to, local=True)):
                self.red_left = True
                self.pair_to = InternalBuoyPair(red_to, green_to)
                self.get_logger().debug("RED BUOYS LEFT, GREEN BUOYS RIGHT")
            else:
                self.red_left = False
                self.pair_to = InternalBuoyPair(green_to, red_to)
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
                self.pair_to = InternalBuoyPair(red_to, green_to)
            else:
                self.pair_to = InternalBuoyPair(green_to, red_to)
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

    def next_pair(self, prev_pair, red, green):
        """
        Returns the next buoy pair from the previous pair,
        by checking the closest one to the middle of the previous buoy pair that's in front of the pair
        """
        front_red = self.filter_front_buoys(prev_pair, red)
        front_green = self.filter_front_buoys(prev_pair, green)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, front red buoys: {self.obs_to_pos(front_red)}, front green buoys: {self.obs_to_pos(front_green)}"
        )

        if not front_red or not front_green:
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
            return None

        if prev_pair is not None:
            prev_pair_midpoint = self.midpoint_pair(prev_pair)
            self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
            # Add a threshold on the minimum distance to a buoy if the new angle on the not close buoys is worse (diagonal but not the one that improves the pair)
            # Instead of only checking the two closest, order by distance and pick either the first one not at duplicate distance or the furthest one
            # If duplicate distance for both reject, if one is duplicate distance check angles
            left_duplicate, left_next = self.pick_buoy(front_red if self.red_left else front_green, prev_pair_midpoint, prev_pair.left)
            right_duplicate, right_next = self.pick_buoy(front_green if self.red_left else front_red, prev_pair_midpoint, prev_pair.right)
            if left_duplicate and right_duplicate:
                return None
            elif (left_duplicate and (not self.check_better_one_side(prev_pair.left, right_next, prev_pair.right))) or (right_duplicate and (not self.check_better_one_side(prev_pair.right, left_next, prev_pair.left))):
                return None
            elif self.norm(self.ob_coords(left_next), self.ob_coords(right_next)) < self.inter_buoy_pair_dist:
                return None
            else:
                return InternalBuoyPair(left_next, right_next)
        else:
            self.get_logger().debug("No previous pair!")
            # TODO: change those to: self.get_closest_to((0,0), red/green, local=True) to not have to use odometry position but just local obstacle positions (wrt the robot)?
            self.get_logger().debug(
                f"next buoys: red: {self.get_closest_to(self.robot_pos, red)}, green: {self.get_closest_to(self.robot_pos, green)}"
            )
            # if there is no previous pair, just take the closest red and green buoys
            if self.red_left:
                return InternalBuoyPair(
                    self.get_closest_to(self.robot_pos, red),
                    self.get_closest_to(self.robot_pos, green),
                )
            else:
                return InternalBuoyPair(
                    self.get_closest_to(self.robot_pos, green),
                    self.get_closest_to(self.robot_pos, red),
                )

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
                 

    def replace_closest(self, ref_obs, obstacles):
        if len(obstacles) == 0:
            return ref_obs, False
        opt_buoy = self.get_closest_to(self.ob_coords(ref_obs), obstacles)
        if self.norm(self.ob_coords(ref_obs), self.ob_coords(opt_buoy)) < self.duplicate_dist:
            return opt_buoy, True
        else:
            return ref_obs, False
        
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
        old_left_duplicate = (self.norm(self.ob_coords(ref_pair.left), self.ob_coords(old_pair.left)))
        old_right_duplicate = (self.norm(self.ob_coords(ref_pair.right), self.ob_coords(old_pair.right)))
        new_left_duplicate = (self.norm(self.ob_coords(ref_pair.left), self.ob_coords(new_pair.left)))
        new_right_duplicate = (self.norm(self.ob_coords(ref_pair.right), self.ob_coords(new_pair.right)))

        old_left = self.get_triangle_angle(ref_pair.left, old_pair.left, old_pair.right)
        old_right = self.get_triangle_angle(ref_pair.right, old_pair.right, old_pair.left)
        new_left = self.get_triangle_angle(ref_pair.left, new_pair.left, new_pair.right)
        new_right = self.get_triangle_angle(ref_pair.right, new_pair.right, new_pair.left)

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
    
    def find_better_pair_to(self, curr_pair, left_buoys, right_buoys):
        changed = False
        new_right = curr_pair.right
        for buoy in right_buoys:
            if self.ob_coords(buoy) == self.ob_coords(curr_pair.right):
                continue
            if self.norm(self.ob_coords(curr_pair.left), self.ob_coords(buoy)) < self.inter_buoy_pair_dist:
                continue
            if not self.ccw((0, 0), self.ob_coords(buoy, local=True), self.ob_coords(curr_pair.left, local=True)):
                continue
            if self.check_better_one_side(curr_pair.left, curr_pair.right, buoy):
                new_right = buoy
                changed = True
        curr_pair.right = new_right
        new_left = curr_pair.left
        for buoy in left_buoys:
            if self.ob_coords(buoy) == self.ob_coords(curr_pair.left):
                continue
            if self.norm(self.ob_coords(curr_pair.right), self.ob_coords(buoy)) < self.inter_buoy_pair_dist:
                continue
            if not self.ccw((0, 0), self.ob_coords(curr_pair.right, local=True), self.ob_coords(buoy, local=True)):
                continue
            if self.check_better_one_side(curr_pair.right, curr_pair.left, buoy):
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
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        self.get_logger().debug(
            f"robot pos: {self.robot_pos}, red buoys: {self.obs_to_pos(red_buoys)}, green buoys: {self.obs_to_pos(green_buoys)}"
        )

        if self.pair_to is None:
            self.get_logger().debug("No pair to go to.")
            self.buoy_pairs = []
            self.pair_to = None
            self.first_buoy_pair = True
            if not self.setup_buoys():
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
            self.buoy_pairs[0].left, res_left_left = self.replace_closest(self.buoy_pairs[0].left, red_buoys if self.red_left else green_buoys)
            self.buoy_pairs[0].right, res_right_right = self.replace_closest(self.buoy_pairs[0].right, green_buoys if self.red_left else red_buoys)
            _, res_left_right = self.replace_closest(self.buoy_pairs[0].left, green_buoys if self.red_left else red_buoys)
            _, res_right_left = self.replace_closest(self.buoy_pairs[0].right, red_buoys if self.red_left else green_buoys)
            # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint, it is false
            if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                self.buoy_pairs = []
                self.pair_to = None
                self.first_buoy_pair = True
                self.get_logger().info('WE ARE GOING TO A FAKE PAIR, FIND PATH AGAIN')
                if not self.setup_buoys():
                    return
            else:
                # Check if new target waypoint is further than adapt_dist away from the old one that's been sent (store it in a global variable and only change it when sending to server)
                if (self.sent_waypoint is not None) and (self.norm(self.midpoint_pair(self.buoy_pairs[0]), self.sent_waypoint) > self.adapt_dist):
                    adapt_waypoint = True
                # changed_pair_to = self.find_better_pair_to(self.buoy_pairs[0], red_buoys if self.red_left else green_buoys, green_buoys if self.red_left else red_buoys)
                self.pair_to = self.buoy_pairs[0]
        ind = 0
        while ind < len(self.buoy_pairs):
            # Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair
            if ind != 0:
                self.buoy_pairs[ind].left, res_left_left = self.replace_closest(self.buoy_pairs[ind].left, red_buoys if self.red_left else green_buoys)
                self.buoy_pairs[ind].right, res_right_right = self.replace_closest(self.buoy_pairs[ind].right, green_buoys if self.red_left else red_buoys)
                _, res_left_right = self.replace_closest(self.buoy_pairs[ind].left, green_buoys if self.red_left else red_buoys)
                _, res_right_left = self.replace_closest(self.buoy_pairs[ind].right, red_buoys if self.red_left else green_buoys)
                # Check if there is not a buoy of the intended color in close distance and there is one from the other color, then remove the waypoint
                if ((not res_left_left) and (res_left_right)) or ((not res_right_right) and (res_right_left)):
                    self.buoy_pairs = self.buoy_pairs[:ind]
                    break
            # Find potential better next pair
            next_pair = self.next_pair(self.buoy_pairs[ind], red_buoys, green_buoys)
            if next_pair is not None and ((ind == (len(self.buoy_pairs)-1) and self.buoy_pairs_distance(self.buoy_pairs[ind], next_pair) > self.buoy_pair_dist_thres) or (ind < (len(self.buoy_pairs)-1) and self.better_buoy_pair_transition(self.buoy_pairs[ind+1],next_pair,self.buoy_pairs[ind]))):
                self.buoy_pairs = self.buoy_pairs[:ind+1]
                self.buoy_pairs.append(next_pair)
            ind += 1

        passed_previous = False
        # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
        self.last_pair = self.pair_to
        left_coords = self.ob_coords(self.pair_to.left)
        right_coords = self.ob_coords(self.pair_to.right)
        x, y = self.midpoint(left_coords, right_coords)
        rx, ry = self.robot_pos
        if self.ccw(
            left_coords,
            right_coords, 
            self.robot_pos,
        ) or (x - rx) ** 2 + (y - ry) ** 2 <= self.thresh_dist:
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
                # TODO: Replace timer with callback on whether we reached the forward waypoint
                if time.time() - self.time_last_seen_buoys > 5:
                    if self.state == FollowPathState.FOLLOWING_FIRST_PASS:
                        self.state = FollowPathState.WAITING_GREEN_BEACON if self.circle_beacon else FollowPathState.FOLLOWING_BACK
                        self.pair_to = self.last_pair
                        self.first_buoy_pair = True
                        self.red_left = not self.red_left
                        self.pair_to.left, self.pair_to.right = self.pair_to.right, self.pair_to.left
                        return
                    else:
                        self.result = True
                        return
                else:
                    if self.sent_forward:
                        return
                    buoy_pair = self.buoy_pairs[0]
                    left_coords = self.ob_coords(buoy_pair.left)
                    right_coords = self.ob_coords(buoy_pair.right)
                    # get the perp forward direction
                    forward_dir = (right_coords[1] - left_coords[1], left_coords[0] - right_coords[0])
                    forward_dir_norm = math.sqrt(forward_dir[0]**2 + forward_dir[1]**2)
                    forward_dir = (-forward_dir[0]/forward_dir_norm, -forward_dir[1]/forward_dir_norm)
                    midpt = self.midpoint_pair(buoy_pair)
                    scale = self.forward_dist
                    wpt = (midpt[0] + scale * forward_dir[0], midpt[1] + scale * forward_dir[1])

                    self.send_waypoint_to_server(wpt)

                    self.get_logger().info("FORWARD WAYPOINT")

                    self.sent_forward = True

                    return
            else:
                if time.time() - self.time_last_seen_buoys > 5:
                    if self.state == FollowPathState.FOLLOWING_FIRST_PASS:
                        self.state = FollowPathState.WAITING_GREEN_BEACON if self.circle_beacon else FollowPathState.FOLLOWING_BACK
                        self.pair_to = self.last_pair
                        self.first_buoy_pair = True
                        self.red_left = not self.red_left
                        self.pair_to.left, self.pair_to.right = self.pair_to.right, self.pair_to.left
                        return
                    else:
                        self.result = True
                        return
        else:
            self.sent_forward = False
            self.time_last_seen_buoys = time.time()
        
        self.waypoints = [self.midpoint_pair(pair) for pair in self.buoy_pairs]
        
        self.get_logger().debug(f"Pairs: {[(self.ob_coords(pair.left), self.ob_coords(pair.right)) for pair in self.buoy_pairs]}")
        self.get_logger().debug(f"Waypoints: {self.waypoints}")

        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(pair.left, pair.right, self.pair_angle_to_pose(
            pair=wpt,
            # angle=(
            #     math.atan(self.ob_coords(pair.right)[1] - self.ob_coords(pair.left)[1]) /
            #     (self.ob_coords(pair.right)[0] - self.ob_coords(pair.left)[0])
            # ) + (math.pi / 2),
            angle=0,
        ), self.norm(self.ob_coords(pair.left), self.ob_coords(pair.right))/2 - self.safe_margin) for wpt, pair in zip(self.waypoints, self.buoy_pairs)]))

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
                self.sent_waypoints.add(waypoint)
                self.first_buoy_pair = False
            elif self.send_goal_future != None and self.lastSelectedGoal != None:
                goal_result = self.send_goal_future.result()
                if self.waypoint_reject or (((goal_result is not None) and (not goal_result.accepted)) or (self.waypoint_sent_future != None and
                                                  self.waypoint_sent_future.result() != None and 
                                                  self.waypoint_sent_future.result().status == GoalStatus.STATUS_ABORTED)):
                    self.get_logger().info("Waypoint request aborted by nav server and no new waypoint option found. Resending request...")
                    self.send_waypoint_to_server(self.lastSelectedGoal)
                    # Waypoint has already been sent before, should be fine to avoid adding it to set?
            if passed_previous:
                self.first_passed_previous = False
        if not passed_previous:
            self.first_passed_previous = True

    def _waypoint_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Strange - sent waypoint rejected immediately.")
            self.waypoint_reject = True
            return
        self.waypoint_sent_future = goal_handle.get_result_async()

    def send_waypoint_to_server(self, waypoint):
        # self.get_logger().info('SENDING WAYPOINT TO SERVER')
        # sending waypoints to navigation server
        self.waypoint_sent_future = None # Reset this... Make sure chance of going backwards is 0
        self.waypoint_reject = False

        self.sent_waypoint = waypoint
        if not self.bypass_planner:
            self.follow_path_client.wait_for_server()
            goal_msg = FollowPath.Goal()
            goal_msg.planner = self.get_parameter("planner").value
            goal_msg.x = waypoint[0]
            goal_msg.y = waypoint[1]
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
            goal_msg.goal_tol = self.get_parameter("goal_tol").value
            goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
            goal_msg.choose_every = self.get_parameter("choose_every").value
            goal_msg.is_stationary = True
            self.follow_path_client.wait_for_server()
            self.send_goal_future = self.follow_path_client.send_goal_async(
                goal_msg
            )
            self.send_goal_future.add_done_callback(self._waypoint_sent_callback)
        else:
            goal_msg = Waypoint.Goal()
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
            goal_msg.x = waypoint[0]
            goal_msg.y = waypoint[1]
            goal_msg.ignore_theta = True
            goal_msg.is_stationary = True
            self.result = False
            self.waypoint_client.wait_for_server()
            self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._waypoint_sent_callback)
        self.lastSelectedGoal = waypoint

    def adapt_pair_to(self):
        green_buoys, red_buoys = self.split_buoys(self.obstacles)
        self.pair_to.left, _ = self.replace_closest(self.pair_to.left, red_buoys if self.red_left else green_buoys)
        self.pair_to.right, _ = self.replace_closest(self.pair_to.right, green_buoys if self.red_left else red_buoys)

    def map_cb(self, msg):
        """
        When a new map is received, check if it is the first one (we haven't set up the starting buoys)
        and find the starting pair, and then (if the starting buoys are successfully computed) form
        the buoy pair / waypoint sequence
        """
        self.obstacles = msg.obstacles

        if self.state in [FollowPathState.WAITING_GREEN_BEACON, FollowPathState.CIRCLING_GREEN_BEACON]:
            self.adapt_pair_to()

    def probe_green_beacon(self):
        '''
        Function to find the green beacon by moving near it (general direction).
        Keeps on appending waypoints to the north/south until it finds 
        '''
        self.get_logger().info("Probing for green beacon")
        max_guide_d = self.beacon_probe_dist
        guide_point = (max_guide_d*self.buoy_direction[0] + self.robot_pos[0], 
                        max_guide_d*self.buoy_direction[1] + self.robot_pos[1])
        self.get_logger().info(f"Current position: {self.robot_pos}. Guide point: {guide_point}.")

        success = self.move_to_point(guide_point, busy_wait=True, abort_func=self.green_beacon_detected)

        return Task.Result(success=success)
    
    def update_green_beacon_pos(self, offset_pos):
        '''
        Updates the position of the green beacon if too far away based on its global map position (and stored previous position) and the offset
        Returns a tuple (update_bool, new_pos) with whether we want to update the goal point and the new point respectively
        '''
        for obstacle in self.obstacles:
            if obstacle.label in self.green_beacon_labels:
                # TODO: perhaps make this check better instead of just checking for a blue circle/buoy (e.g. make it pick closest one or smth)
                self.green_beacon_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
                if self.norm(self.green_beacon_pos, self.prev_sent_beacon_pos) > self.circle_adapt_dist:
                    self.prev_sent_beacon_pos = self.green_beacon_pos
                    return (True, (self.green_beacon_pos[0]+offset_pos[0], self.green_beacon_pos[1]+offset_pos[1]))
        return (False, None)

    def move_to_point(self, point, is_stationary=False, busy_wait=False, abort_func=None, goal_update_func=None):
        '''
        Moves the boat to the specified position using the follow path action server.
        # Returns the future of the server request.

        Busy waits until the boat moved to the point (bad, should be fixed with asyncio patterns)

        Returns true if aborted by the function
        Sends new waypoint if desired by the goal_update_func
        '''
        self.get_logger().info(f"Moving to point {point}")
        self.moved_to_point = False
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

        self.follow_path_client.wait_for_server()
        self.send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg
        )
        self._get_result_future = None
        self.waypoint_reject = False
        self.send_goal_future.add_done_callback(self.follow_path_response_cb)
        if busy_wait:
            while not self.moved_to_point:
                if (abort_func is not None) and abort_func():
                    return True
                if (goal_update_func is not None):
                    update_goal, new_goal = goal_update_func()
                    if update_goal:
                        goal_msg.x = new_goal[0]
                        goal_msg.y = new_goal[1]

                        self.get_logger().info('ADAPTING GOAL POINT')
                        self.follow_path_client.wait_for_server()
                        self.send_goal_future = self.follow_path_client.send_goal_async(
                            goal_msg
                        )
                        self._get_result_future = None
                        self.send_goal_future.add_done_callback(self.follow_path_response_cb)

                goal_result = self.send_goal_future.result()
                if self.waypoint_reject or (((goal_result is not None) and (not goal_result.accepted)) or (self._get_result_future != None and
                                                  self._get_result_future.result() != None and 
                                                  self._get_result_future.result().status == GoalStatus.STATUS_ABORTED)):
                    self.get_logger().info('RESENDING GOAL')
                    self.follow_path_client.wait_for_server()
                    self.send_goal_future = self.follow_path_client.send_goal_async(
                        goal_msg
                    )
                    self._get_result_future = None
                    self.send_goal_future.add_done_callback(self.follow_path_response_cb)
                time.sleep(self.timer_period)
        return False

    def move_to_waypoint(self, point, is_stationary=False, busy_wait=False, abort_func=None, goal_update_func=None):
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
        self.result = False
        self.waypoint_client.wait_for_server()
        self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.follow_path_response_cb)
        if busy_wait:
            while not self.moved_to_point:
                if (abort_func is not None) and abort_func():
                    return True
                if (goal_update_func is not None):
                    update_goal, new_goal = goal_update_func()
                    if update_goal:
                        goal_msg.x = new_goal[0]
                        goal_msg.y = new_goal[1]

                        self.get_logger().info('ADAPTING WAYPOINT')
                        self.result = False
                        self.waypoint_client.wait_for_server()
                        self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
                        self.send_goal_future.add_done_callback(self.follow_path_response_cb)
                time.sleep(self.timer_period)
        return False
        
    def follow_path_response_cb(self, future):
        '''
        Responds to follow path action server goal response.
        '''
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Waypoint rejected')
            self.waypoint_reject = True
            return

        self.get_logger().info("Waypoint accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_point_result_cb)

    def get_point_result_cb(self, future):
        '''
        Flags the path following as complete for move_to_point
        '''
        # Marks path following as finished/ moved to path following point
        # if path following is interrupted, does not affect moved to point
        result = future.result().result
        if result.is_finished:
            self.moved_to_point = True

    def green_beacon_detected(self):
        '''
        Check if the green beacon for turning is detected (returns boolean).
        Also sets the position of the green beacon if it is found.
        '''
        for obstacle in self.obstacles:
            if obstacle.label in self.green_beacon_labels:
                # TODO: perhaps make this check better instead of just checking for a blue circle/buoy (e.g. make it pick closest one or smth)
                self.get_logger().info(f"Found green beacon at {obstacle.global_point.point}")
                self.green_beacon_found = True
                self.green_beacon_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
                robot_x, robot_y = self.robot_pos
                robot_buoy_vector = (self.green_beacon_pos[0]-robot_x, self.green_beacon_pos[1]-robot_y)
                robot_buoy_dist = self.norm(robot_buoy_vector)
                self.buoy_direction = (robot_buoy_vector[0]/robot_buoy_dist, robot_buoy_vector[1]/robot_buoy_dist)
                break
        return self.green_beacon_found

    def circle_green_beacon(self):
        '''
        Function to circle the green beacon.
        '''
        self.get_logger().info("Circling green beacon")
        if not self.green_beacon_detected():
            self.get_logger().info("speed challenge probing exited without finding green beacon")
            return Task.Result(success=False)
        
        # circle the green beacon like a baseball diamond
        # a better way to do this might be to have the astar run to original cell, 
        # but require the path to go around buoy

        t_o = self.turn_offset
        first_dir = (self.buoy_direction[1]*t_o, -self.buoy_direction[0]*t_o)
        second_dir = (self.buoy_direction[0]*t_o, self.buoy_direction[1]*t_o)
        third_dir = (-first_dir[0], -first_dir[1])


        add_tuple = lambda a,b: tuple(sum(x) for x in zip(a, b))
        first_base = add_tuple(self.green_beacon_pos, first_dir)
        second_base = add_tuple(self.green_beacon_pos, second_dir)
        third_base = add_tuple(self.green_beacon_pos, third_dir)

        bases = [first_base, second_base, third_base]
        self.get_logger().info(f"initial moved to points= {self.moved_to_point}")
        self.get_logger().info(f"green beacon pose: {self.green_beacon_pos}")
        self.get_logger().info(f"bases: {bases}")
        self.prev_sent_beacon_pos = self.green_beacon_pos
        for base, offset in zip(bases, [first_dir, second_dir, third_dir]):
            self.move_to_point(base, busy_wait=True, goal_update_func=partial(self.update_green_beacon_pos, offset))
            self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)
    
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

        self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True)

        # LOOK RIGHT 30 DEG

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True)

        # BACK TO FORWARD

        self.move_to_waypoint([nav_x, nav_y, heading], is_stationary=True, busy_wait=False)

    def search_buoys(self):
        nav_x, nav_y, heading = self.get_robot_pose()

        # LOOK LEFT 30 DEG

        self.get_logger().info(f"Turning left")

        self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True)

        # LOOK RIGHT 30 DEG

        self.get_logger().info(f"Turning right")

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True)

    def search_beacon(self):
        nav_x, nav_y, heading = self.get_robot_pose()

        # # LOOK LEFT 30 DEG

        # self.get_logger().info(f"Turning left")

        # self.move_to_waypoint([nav_x, nav_y, heading + (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True)

        # LOOK RIGHT 30 DEG

        # self.get_logger().info(f"Turning right")

        self.move_to_waypoint([nav_x, nav_y, heading - (30.0 * 2 * math.pi / 360)], is_stationary=False, busy_wait=True, abort_func=self.green_beacon_detected)

    def execute_callback(self, goal_handle):

        self.start_process("Follow buoy path started!")

        # self.station_hold()

        while rclpy.ok() and self.obstacles is None:
            time.sleep(self.timer_period)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
        
        success = False
        while not success:
            success = self.setup_buoys()
            time.sleep(self.timer_period)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()

        self.get_logger().info("Setup buoys succeeded!")

        self.state = FollowPathState.FOLLOWING_FIRST_PASS

        while not self.result:
            # Check if we should abort/cancel if a new goal arrived
            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Cancel requested. Aborting path following.")
                goal_handle.canceled()
                return Task.Result()
            
            if self.state in [FollowPathState.FOLLOWING_FIRST_PASS, FollowPathState.FOLLOWING_BACK]:
                if self.state == FollowPathState.FOLLOWING_BACK:
                    if "green_pole_buoy" in self.green_labels:
                        self.green_labels.remove("green_pole_buoy")
                    if "red_pole_buoy" in self.red_labels:
                        self.red_labels.remove("red_pole_buoy")
                self.generate_waypoints()
            elif self.state == FollowPathState.WAITING_GREEN_BEACON:
                self.get_logger().info(f"Searching green beacon")
        
                self.search_beacon()
                
                self.get_logger().info(f'Detecting green beacon')
                self.home_pos = self.robot_pos # keep track of home position
                self.buoy_direction = self.robot_dir
                self.get_logger().info(f"Facing direction: {self.buoy_direction}")
                action_result = self.probe_green_beacon()
                if action_result.success == False:
                    self.state = FollowPathState.FOLLOWING_BACK
                else:
                    self.state = FollowPathState.CIRCLING_GREEN_BEACON
            elif self.state == FollowPathState.CIRCLING_GREEN_BEACON:
                self.get_logger().info(f'Circling green beacon')
                action_result = self.circle_green_beacon()
                # action_result = self.return_to_start()
                self.state = FollowPathState.FOLLOWING_BACK

            time.sleep(self.timer_period)

        self.end_process("Follow buoy path completed!")
        goal_handle.succeed()
        return Task.Result(success=True)


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
