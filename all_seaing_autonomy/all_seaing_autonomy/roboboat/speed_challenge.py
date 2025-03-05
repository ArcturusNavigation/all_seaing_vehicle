from ast import Num
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle, LabeledBoundingBox2DArray, LabeledBoundingBox2D
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase

import math
import os
import yaml
import time

# class InternalBuoyPair:
#     def __init__(self, left_buoy=None, right_buoy=None):
#         if left_buoy is None:
#             self.left = Obstacle()
#         else:
#             self.left = left_buoy

#         if right_buoy is None:
#             self.right = Obstacle()
#         else:
#             self.right = right_buoy


class SpeedChange(ActionServerBase):
    def __init__(self):
        super().__init__("speed_challenge_server")

        self._action_server = ActionServer(
            self,
            Task,
            "speed_challenge",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        self.seg_image_bbox_sub = self.create_subscription(
            LabeledBoundingBox2D, "bounding_boxes", self.bbox_cb, 10
        )

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/labeled", self.map_cb, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 5)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.robot_pos = (0, 0)
        self.home_pos = (0, 0)
        self.runnerActivated = False

        self.declare_parameter("safe_margin", 0.2)

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

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
            self.color_label_mappings = yaml.safe_load(f)

        # self.result = False
        # self.timer_period = 1/60
        self.time_last_seen_buoys = time.time()

        self.buoy_pairs = []
        self.obstacles = []

    def bbox_cb(self):
        '''
        Handles when an color segmented image gets published
        '''
        if self.runnerActivated:
            return

        ###### checks if the color segmented image depicts the LED changing from red to green
        ###### if so, make self.runnerActivated to be true.

        if self.runnerActivated:
            self.home_pos = self.robot_pos # keep track of where the home position is
            self.circle_blue_buoy()

    def circle_blue_buoy(self):
        '''
        Function to circle the blue buoy
        '''

    # robust realtime visual signal processing
    def go(self, beforeRed, afterRed, beforeGreen, afterGreen, epsilon, lmbda, p, limit):
        '''
        beforeRed :: [[Bbox]] | len(beforeRed) > 0
        frames with red bounding boxes before signal event

        afterRed :: [[Bbox]] | len(afterRed) > 0
        frames with red bounding boxes after signal event

        beforeGreen :: [[Bbox]] | len(beforeGreen) = len(beforeRed) > 0
        same as `beforeRed` but for green bounding boxes

        afterGreen :: [[Bbox]] | len(afterGreen) = len(afterRed) > 0
        same as `afterRed` but for green bounding boxes

        epsilon :: Num
        maximum distance at which two bounding boxes are considered the same;
        also minimum distance needed to identify bounding boxes as unique

        lmda :: Num
        maximum deviation in position of a bounding box across frames

        p :: Int
        number of frames to sample from full list to determine main bounding boxes;
        should be small for performance reasons

        limit :: Prob
        threshold probability of existence required to detect a change in bounding box color

        → Bool
        whether signal has changed

        ---

        Bbox := {
            x :: Num
            y :: Num
            w :: Num | w > 0
            h :: Num | h > 0
        }

        Prob := Num a | 0 <= a <= 1
        '''
        id = lambda x: x  # A → A
        map = lambda f, l: [f(x) for x in l]  # (A → B) → [A] → [B]
        flatten = lambda l: l[0] + flatten(l[1:]) if len(l) > 0 else []  # [[A]] → [A]
        filter = lambda p, l: [x for x in l if p(x)]  # (A → Bool) → [A] → [A]
        product = lambda a, b: flatten([[(alpha, beta) for alpha in a] for beta in b])  # [A] → [B] → [(A, B)]
        access = lambda l1, l2: [l1[i] for i in l2]  # [A] → [Int] → [A]
        distinguishable = lambda dist: dist > epsilon  # Num → Bool
        matching = lambda dist: dist < lmbda  # Num → Bool
        changed = lambda prob: prob >= limit  # Prob → Bool
        multiply = lambda x: x[0] * x[1]  # (Num, Num) → Num
        norm = lambda b1: lambda b2: (b1.x - b2.x) ** 2 + (b1.y - b2.y) ** 2  # Num → (Num → Num)
        estimator = lambda trials: len(filter(id, trials)) / len(trials)  # [Bool] → Num

        # [Bbox] → (Bbox → Bool)
        # whether the bounding box is distinguishable from all bounding boxes in a list of bounding boxes
        distinct = lambda boxes: lambda box: all(map(distinguishable, map(norm(box), boxes)))

        # [[Bbox]] → [[Bbox]]
        # remove red bounding boxes that are indistinguishable from green ones
        beforeCandidatesRed = filter(distinct(beforeGreen), beforeRed)

        # Int | nBefore > 0
        # number of "before" frames
        nBefore = len(beforeRed)

        # Int | nAfter > 0
        # number of "after" frames
        nAfter = len(afterRed)

        # Int → Int → Int | n > 0 → [Int]
        # generate `n` evenly spaced elements from an arbitrary discrete range
        linspace = lambda a, b, n: [a] + linspace(a + (b-a)/(n-1), b, n-1) if n > 1 else [a]

        # [Int]
        # indices of "before" frames to sample
        beforeSampleIndices = linspace(0, nBefore-1, p)

        # [[Bbox]]
        # bounding box data for red before frames
        beforeSampleRed = access(beforeCandidatesRed, beforeSampleIndices)

        # [Bbox]
        # list of identified red bounding boxes from sample frames
        redBboxes = flatten(beforeSampleRed)

        # Bbox → ([Bbox] → Bool)
        # detect whether a particular bounding box exists in a different frame
        bboxExists = lambda bbox: lambda frame: any(map(matching, map(norm(bbox), frame)))

        # Bbox → [[Bbox]] → Prob
        # determine probability that a given bounding box exists in time series
        probExistence = lambda box, series: estimator(map(bboxExists(box), series))

        # [Int]
        # indices of "after" frames to sample
        afterSampleIndices = linspace(0, nAfter-1, p)

        # [[Bbox]]
        # bounding box data for green after frames
        afterSampleGreen = access(afterGreen, afterSampleIndices)

        # [Bbox]
        # list of identified green bounding boxes from sample frames
        greenBoxes = flatten(afterSampleGreen)

        # (Bbox, Bbox) → Bool
        # check for overlap between bounding box set and a single bounding box
        overlapping = lambda boxes: matching(norm(boxes[0])(boxes[1]))

        # [(Bbox, Bbox)]
        # list of potential candidates for a signal switch
        signalCandidates = filter(overlapping, product(redBboxes, greenBoxes))

        # (Bbox, Bbox) → Prob
        # probability of specific candidate pair existing
        candidateProbability = lambda boxes: (probExistence(boxes[0], beforeRed), probExistence(boxes[1], afterGreen))

        # [(Prob, Prob)]
        # probability of existence of each bounding box in signal candidates
        probabilities = map(candidateProbability, signalCandidates)

        # [Prob]
        # probability of existence of each bounding box pair in signal candidates
        pairProbabilities = map(multiply, probabilities)

        # check for any probability exceeding confidence threshold
        return any(map(changed, pairProbabilities))

    # def norm_squared(self, vec, ref=(0, 0)):
    #     return vec[0] ** 2 + vec[1] ** 2

    # def norm(self, vec, ref=(0, 0)):
    #     return math.sqrt(self.norm_squared(vec, ref))

    # def ob_coords(self, buoy, local=False):
    #     if local:
    #         return (buoy.local_point.point.x, buoy.local_point.point.y)
    #     else:
    #         return (buoy.global_point.point.x, buoy.global_point.point.y)

    # def get_closest_to(self, source, buoys, local=False):
    #     return min(
    #         buoys,
    #         key=lambda buoy: math.dist(source, self.ob_coords(buoy, local)),
    #     )

    # def midpoint(self, vec1, vec2):
    #     return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    # def midpoint_pair(self, pair):
    #     left_coords = self.ob_coords(pair.left)
    #     right_coords = self.ob_coords(pair.right)
    #     midpoint = self.midpoint(left_coords, right_coords)

    #     scale = 1 # number of meters to translate forward. TODO: parametrize.
    #     dy = right_coords[1] - left_coords[1]
    #     dx = right_coords[0] - left_coords[0]
    #     norm = math.sqrt(dx**2 + dy**2)
    #     dx /= norm
    #     dy /= norm
    #     midpoint = (midpoint[0] - scale*dy, midpoint[1] + scale*dx)

    #     return midpoint

    # def split_buoys(self, obstacles):
    #     """
    #     Splits the buoys into red and green based on their labels in the obstacle map
    #     """
    #     green_bouy_points = []
    #     red_bouy_points = []
    #     for obstacle in obstacles:
    #         if obstacle.label == self.color_label_mappings["green"]:
    #             green_bouy_points.append(obstacle)
    #         elif obstacle.label == self.color_label_mappings["red"]:
    #             red_bouy_points.append(obstacle)
    #     return green_bouy_points, red_bouy_points

    # def obs_to_pos(self, obs):
    #     return [self.ob_coords(ob, local=False) for ob in obs]

    # def obs_to_pos_label(self, obs):
    #     return [self.ob_coords(ob, local=False) + (ob.label,) for ob in obs]

    # def buoy_pairs_to_markers(self, buoy_pairs):
    #     """
    #     Create the markers from an array of buoy pairs to visualize them (and the respective waypoints) in RViz
    #     """
    #     marker_array = MarkerArray()
    #     i = 0
    #     for p_left, p_right, point, radius in buoy_pairs:
    #         marker_array.markers.append(
    #             Marker(
    #                 type=Marker.ARROW,
    #                 pose=point,
    #                 header=Header(frame_id="odom"),
    #                 scale=Vector3(x=1.0, y=0.05, z=0.05),
    #                 color=ColorRGBA(a=1.0),
    #                 id=(4 * i),
    #             )
    #         )
    #         if self.red_left:
    #             left_color = ColorRGBA(r=1.0, a=1.0)
    #             right_color = ColorRGBA(g=1.0, a=1.0)
    #         else:
    #             left_color = ColorRGBA(g=1.0, a=1.0)
    #             right_color = ColorRGBA(r=1.0, a=1.0)

    #         marker_array.markers.append(
    #             Marker(
    #                 type=Marker.SPHERE,
    #                 pose=self.pair_to_pose(self.ob_coords(p_left)),
    #                 header=Header(frame_id="odom"),
    #                 scale=Vector3(x=1.0, y=1.0, z=1.0),
    #                 color=left_color,
    #                 id=(4 * i) + 1,
    #             )
    #         )
    #         marker_array.markers.append(
    #             Marker(
    #                 type=Marker.SPHERE,
    #                 pose=self.pair_to_pose(self.ob_coords(p_right)),
    #                 header=Header(frame_id="odom"),
    #                 scale=Vector3(x=1.0, y=1.0, z=1.0),
    #                 color=right_color,
    #                 id=(4 * i) + 2,
    #             )
    #         )
    #         marker_array.markers.append(
    #             Marker(
    #                 type=Marker.CYLINDER,
    #                 pose=Pose(
    #                     position=Point(
    #                         x=point.position.x,
    #                         y=point.position.y,
    #                     )
    #                 ),
    #                 header=Header(frame_id="odom"),
    #                 scale=Vector3(
    #                     x=radius, y=radius, z=1.0
    #                 ),
    #                 color=ColorRGBA(g=1.0, a=0.5),
    #                 id=(4 * i) + 3,
    #             )
    #         )
    #         i += 1
    #     return marker_array

    # def setup_buoys(self):
    #     """
    #     Runs when the first obstacle map is received, filters the buoys that are in front of
    #     the robot (x>0 in local coordinates) and finds (and stores) the closest green one and
    #     the closest red one, and because the robot is in the starting position these
    #     are the front buoys of the robot starting box.
    #     """
    #     self.get_logger().debug("Setting up starting buoys!")
    #     self.get_logger().debug(
    #         f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
    #     )

    #     # Split all the buoys into red and green
    #     green_init, red_init = self.split_buoys(self.obstacles)

    #     # lambda function that filters the buoys that are in front of the robot
    #     obstacles_in_front = lambda obs: [
    #         ob for ob in obs
    #         if (self.is_sim and ob.local_point.point.x > 0) or (not self.is_sim and ob.local_point.point.y > 0)
    #     ]
    #     # take the green and red buoys that are in front of the robot
    #     green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
    #     self.get_logger().debug(
    #         f"initial red buoys: {red_buoys}, green buoys: {green_buoys}"
    #     )
    #     if len(red_buoys) == 0 or len(green_buoys) == 0:
    #         self.get_logger().debug("No starting buoy pairs!")
    #         return False

    #     # From the red buoys that are in front of the robot, take the one that is closest to it.
    #     # And do the same for the green buoys.
    #     # This pair is the front pair of the starting box of the robot.
    #     closest_red = self.get_closest_to((0, 0), red_buoys, local=True)
    #     closest_green = self.get_closest_to((0, 0), green_buoys, local=True)
    #     if self.ccw(
    #         (0, 0),
    #         self.ob_coords(closest_green, local=True),
    #         self.ob_coords(closest_red, local=True),
    #     ):
    #         self.red_left = True
    #         self.starting_buoys = InternalBuoyPair(
    #             self.get_closest_to((0, 0), red_buoys, local=True),
    #             self.get_closest_to((0, 0), green_buoys, local=True),
    #         )
    #         self.get_logger().info("RED BUOYS LEFT, GREEN BUOYS RIGHT")
    #     else:
    #         self.red_left = False
    #         self.starting_buoys = InternalBuoyPair(
    #             self.get_closest_to((0, 0), green_buoys, local=True),
    #             self.get_closest_to((0, 0), red_buoys, local=True),
    #         )
    #         self.get_logger().info("GREEN BUOYS LEFT, RED BUOYS RIGHT")
    #     self.pair_to = self.starting_buoys
    #     # self.backup_pair = None
    #     return True

    # def ccw(self, a, b, c):
    #     """Return True if the points a, b, c are counterclockwise, respectively"""
    #     area = (
    #         a[0] * b[1]
    #         + b[0] * c[1]
    #         + c[0] * a[1]
    #         - a[1] * b[0]
    #         - b[1] * c[0]
    #         - c[1] * a[0]
    #     )
    #     return area > 0

    # def filter_front_buoys(self, pair, buoys):
    #     """
    #     Returns the buoys (from the given array) that are in front of a pair of points,
    #     considering the forward direction to be the one such that
    #     the first point of the pair is in the left and the second is in the right
    #     """
    #     # (red, green)
    #     return [
    #         buoy
    #         for buoy in buoys
    #         if self.ccw(
    #             self.ob_coords(pair.left),
    #             self.ob_coords(pair.right),
    #             self.ob_coords(buoy),
    #         ) and min(self.norm((self.ob_coords(buoy)[0]-self.ob_coords(pair.left)[0], self.ob_coords(buoy)[1]-self.ob_coords(pair.left)[1])), self.norm((self.ob_coords(buoy)[0]-self.ob_coords(pair.right)[0], self.ob_coords(buoy)[1]-self.ob_coords(pair.right)[1]))) > self.buoy_pair_dist_thres
    #     ]

    # def next_pair(self, prev_pair, red, green):
    #     """
    #     Returns the next buoy pair from the previous pair,
    #     by checking the closest one to the middle of the previous buoy pair that's in front of the pair
    #     """

    #     front_red = self.filter_front_buoys(prev_pair, red)
    #     front_green = self.filter_front_buoys(prev_pair, green)
    #     self.get_logger().debug(
    #         f"robot pos: {self.robot_pos}, front red buoys: {self.obs_to_pos(front_red)}, front green buoys: {self.obs_to_pos(front_green)}"
    #     )

    #     if not front_red or not front_green:
    #         prev_coords = self.ob_coords(prev_pair.left), self.ob_coords(
    #             prev_pair.right
    #         )
    #         red_coords = self.obs_to_pos(red)
    #         green_coords = self.obs_to_pos(green)

    #         self.get_logger().debug(
    #             f"buoys:  {prev_coords} \nred: {red_coords} \ngreen: {green_coords}"
    #         )
    #         self.get_logger().debug(f"robot: {self.robot_pos}")
    #         self.get_logger().debug("Missing at least one front buoy")
    #         return None

    #     if prev_pair is not None:
    #         prev_pair_midpoint = self.midpoint_pair(prev_pair)
    #         self.get_logger().debug(f"prev pair midpoint: {prev_pair_midpoint}")
    #         if self.red_left:
    #             return InternalBuoyPair(
    #                 self.get_closest_to(prev_pair_midpoint, front_red),
    #                 self.get_closest_to(prev_pair_midpoint, front_green),
    #             )
    #         else:
    #             return InternalBuoyPair(
    #                 self.get_closest_to(prev_pair_midpoint, front_green),
    #                 self.get_closest_to(prev_pair_midpoint, front_red),
    #             )
    #     else:
    #         self.get_logger().debug("No previous pair!")
    #         # TODO: change those to: self.get_closest_to((0,0), red/green, local=True) to not have to use odometry position but just local obstacle positions (wrt the robot)?
    #         self.get_logger().debug(
    #             f"next buoys: red: {self.get_closest_to(self.robot_pos, red)}, green: {self.get_closest_to(self.robot_pos, green)}"
    #         )
    #         # if there is no previous pair, just take the closest red and green buoys
    #         if self.red_left:
    #             return InternalBuoyPair(
    #                 self.get_closest_to(self.robot_pos, red),
    #                 self.get_closest_to(self.robot_pos, green),
    #             )
    #         else:
    #             return InternalBuoyPair(
    #                 self.get_closest_to(self.robot_pos, green),
    #                 self.get_closest_to(self.robot_pos, red),
    #             )

    # def pair_to_pose(self, pair):
    #     return Pose(position=Point(x=pair[0], y=pair[1]))

    # def quaternion_from_euler(self, roll, pitch, yaw):
    #     """
    #     Converts euler roll, pitch, yaw to quaternion (w in last place)
    #     quat = [x, y, z, w]
    #     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    #     """
    #     cy = math.cos(yaw * 0.5)
    #     sy = math.sin(yaw * 0.5)
    #     cp = math.cos(pitch * 0.5)
    #     sp = math.sin(pitch * 0.5)
    #     cr = math.cos(roll * 0.5)
    #     sr = math.sin(roll * 0.5)

    #     q = [0] * 4
    #     q[0] = cy * cp * cr + sy * sp * sr
    #     q[1] = cy * cp * sr - sy * sp * cr
    #     q[2] = sy * cp * sr + cy * sp * cr
    #     q[3] = sy * cp * cr - cy * sp * sr

    #     return q

    # def pair_angle_to_pose(self, pair, angle):
    #     quat = self.quaternion_from_euler(0, angle, 0)
    #     return Pose(
    #         position=Point(x=pair[0], y=pair[1]),
    #         orientation=Quaternion(x=quat[0], y=quat[2], z=quat[2], w=quat[3]),
    #     )

    # def buoy_pairs_angle(self, p1, p2, loc=False):
    #     p1_left, p1_right = (self.ob_coords(p1.left, local=loc), self.ob_coords(p1.right, local=loc))
    #     p2_left, p2_right = (self.ob_coords(p2.left, local=loc), self.ob_coords(p2.right, local=loc))
    #     p1_diff = (p1_right[0]-p1_left[0], p1_right[1]-p1_left[1])
    #     p2_diff = (p2_right[0]-p2_left[0], p2_right[1]-p2_left[1])
    #     angle = math.acos((p1_diff[0]*p2_diff[0]+p1_diff[1]*p2_diff[1])/(self.norm(p1_diff)*self.norm(p2_diff)))
    #     return angle

    # def buoy_pairs_distance(self, p1, p2, mode="min", loc=False):
    #     p1_left, p1_right = self.ob_coords(p1.left, local=loc), self.ob_coords(p1.right, local=loc)
    #     p2_left, p2_right = self.ob_coords(p2.left, local=loc), self.ob_coords(p2.right, local=loc)
    #     if mode=="mid":
    #         p1_mid = ((p1_right[0]+p1_left[0])/2.0, (p1_right[1]+p1_left[1])/2.0)
    #         p2_mid = ((p2_right[0]+p2_left[0])/2.0, (p2_right[1]+p2_left[1])/2.0)
    #         dist = self.norm((p2_mid[0]-p1_mid[0], p2_mid[1]-p1_mid[1]))
    #     else:
    #         left_diff = (p2_left[0]-p1_left[0], p2_left[1]-p1_left[1])
    #         right_diff = (p2_right[0]-p1_right[0], p2_right[1]-p1_right[1])
    #         dist = min(self.norm(left_diff), self.norm(right_diff))
    #     return dist

    # def better_buoy_pair_transition(self, p_old, p_new, p_ref):
    #     # return ((self.buoy_pairs_distance(p_ref, p_new) <= self.buoy_pair_dist_thres and
    #     #         self.buoy_pairs_distance(p_ref, p_new, mode="min") > self.buoy_pairs_distance(p_ref, p_old, mode="min")) or
    #     return (self.buoy_pairs_distance(p_ref, p_new) > self.buoy_pair_dist_thres and
    #             (self.buoy_pairs_distance(p_ref, p_old) <= self.buoy_pair_dist_thres or
    #              self.buoy_pairs_angle(p_ref, p_old) > self.buoy_pairs_angle(p_ref, p_new)))

    # def generate_waypoints(self):
    #     """
    #     Runs every time a new obstacle map is received, keeps
    #     track of the pair of buoys the robot is heading towards,
    #     checks if it passed it (the robot is in front of the pair of buoys)
    #     (#TODO: add a margin of error such that the robot is considered to have passed the buoys if it's
    #     a bit in front of them) and update the pair accordingly, and afterwards computes
    #     the sequence of future waypoints based on the first waypoint
    #     and the next_pair() function to compute the next pair from each one in the sequence,
    #     as long as there is a next pair from the buoys that are stored in the obstacle map.
    #     """
    #     # split the buoys into red and green
    #     green_buoys, red_buoys = self.split_buoys(self.obstacles)
    #     self.get_logger().debug(
    #         f"robot pos: {self.robot_pos}, red buoys: {self.obs_to_pos(red_buoys)}, green buoys: {self.obs_to_pos(green_buoys)}"
    #     )

    #     if self.pair_to is None:
    #         self.get_logger().debug("No pair to go to.")
    #         return

    #     # TODO: Match the previous pair of buoys to the new obstacle map (in terms of global position) to eliminate any big drift that may mess up the selection of the next pair

    #     """
    #     Update the current sequence if better transitions are found
    #     """
    #     ind = 0
    #     while ind < len(self.buoy_pairs):
    #         next_pair = self.next_pair(self.buoy_pairs[ind], red_buoys, green_buoys)
    #         if next_pair is not None and ((ind == (len(self.buoy_pairs)-1) and self.buoy_pairs_distance(self.buoy_pairs[ind], next_pair) > self.buoy_pair_dist_thres) or (ind < (len(self.buoy_pairs)-1) and self.better_buoy_pair_transition(self.buoy_pairs[ind+1],next_pair,self.buoy_pairs[ind]))):
    #             self.buoy_pairs = self.buoy_pairs[:ind+1]
    #             self.waypoints = self.waypoints[:ind+1]
    #             self.buoy_pairs.append(next_pair)
    #             self.waypoints.append(self.midpoint_pair(next_pair))
    #         ind += 1

    #     passed_previous = False
    #     # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
    #     if self.ccw(
    #         self.ob_coords(self.pair_to.left),
    #         self.ob_coords(self.pair_to.right),
    #         self.robot_pos,
    #     ):
    #         passed_previous = True
    #         # new_pair = self.next_pair(self.pair_to, red_buoys, green_buoys)
    #         # if new_pair is not None:
    #         #     self.pair_to = new_pair
    #         #     self.time_last_seen_buoys = time.time()
    #         # else:
    #         #     # if self.backup_pair is not None:
    #         #     #     self.get_logger().info("Using previously seen backup pair")
    #         #     #     self.pair_to = self.backup_pair
    #         #     #     self.backup_pair = None
    #         #     # else:
    #         #     self.get_logger().debug("No next buoy pair to go to.")
    #         #     if time.time() - self.time_last_seen_buoys > 1:
    #         #         self.result = True
    #         #     return

    #     if self.first_buoy_pair:
    #         self.buoy_pairs = [self.pair_to]
    #         self.waypoints = [self.midpoint_pair(self.pair_to)]
    #     elif passed_previous:
    #         if len(self.buoy_pairs)>=2:
    #             self.buoy_pairs = self.buoy_pairs[1:]
    #             self.waypoints = self.waypoints[1:]
    #             self.time_last_seen_buoys = time.time()
    #             self.pair_to = self.buoy_pairs[0]
    #         else:
    #             # below is equivalent to the previous case as we update the sequence before we go to the next buoy pair
    #             # if self.next_pair(self.pair_to, red_buoys, green_buoys) != None:
    #             #     self.get_logger().info("FOUND NEW NEXT BUOY PAIR")
    #             #     self.pair_to = self.next_pair(self.pair_to, red_buoys, green_buoys)
    #             #     self.buoy_pairs = [self.pair_to]
    #             #     self.waypoints = [self.midpoint_pair(self.pair_to)]
    #             #     self.time_last_seen_buoys = time.time()
    #             # else:
    #             self.get_logger().debug("No next buoy pair to go to.")
    #             if time.time() - self.time_last_seen_buoys > 1:
    #                 self.result = True
    #             return

    #     # self.get_logger().debug(f"pair to: {len(self.buoy_pairs)}")

    #     # """
    #     # Form a sequence of buoy pairs (and the respective waypoints) that form a path that the robot can follow
    #     # will terminate if we run out of either green or red buoys
    #     # """

    #     # next_buoy_pair = self.next_pair(self.buoy_pairs[-1], red_buoys, green_buoys)
    #     # while next_buoy_pair is not None:
    #     #     self.buoy_pairs.append(next_buoy_pair)
    #     #     self.waypoints.append(self.midpoint_pair(next_buoy_pair))
    #     #     next_buoy_pair = self.next_pair(self.buoy_pairs[-1], red_buoys, green_buoys)

    #     # if(len(self.buoy_pairs)>=2):
    #     #     # choose the next pair with the least angle compared to the current one as a backup, to hopefully mitigate diagonal ones caused by duplicates, also take distance into account
    #     #     if self.backup_pair is None or self.better_buoy_pair_transition(self.backup_pair, self.buoy_pairs[1], self.pair_to):
    #     #         self.backup_pair = self.buoy_pairs[1]
    #     #         self.get_logger().info(f"Updated backup buoy pair with angle: {self.buoy_pairs_angle(self.pair_to, self.backup_pair)} and distance: {self.buoy_pairs_distance(self.pair_to, self.backup_pair)}")

    #     self.get_logger().debug(f"Waypoints: {self.waypoints}")

    #     self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
    #     self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(pair.left, pair.right, self.pair_angle_to_pose(
    #         pair=wpt,
    #         angle=(
    #             math.atan(self.ob_coords(pair.right)[1] - self.ob_coords(pair.left)[1]) /
    #             (self.ob_coords(pair.right)[0] - self.ob_coords(pair.left)[0])
    #         ) + (math.pi / 2),
    #     ), self.norm(self.ob_coords(pair.left), self.ob_coords(pair.right))/2 - self.safe_margin) for wpt, pair in zip(self.waypoints, self.buoy_pairs)]))

    #     if self.waypoints:
    #         waypoint = self.waypoints[0]
    #         self.get_logger().debug(
    #             f"cur_waypoint: {waypoint}, sent_waypoints: {self.sent_waypoints}"
    #         )
    #         self.get_logger().debug(f"len(waypoints): {len(self.waypoints)}")

    #         # check if waypoint is close enough (check_dist) to some previous waypoint
    #         # passed_waypoint = False
    #         # check_dist = 5.0  # TODO: FIX magic number T_T
    #         # for sent_waypoint in self.sent_waypoints:
    #         #     if math.dist(waypoint, sent_waypoint) < check_dist:
    #         #         passed_waypoint = True

    #         # buoy_pair = buoy_pairs[0]
    #         # left_coords = self.ob_coords(buoy_pair.left)
    #         # right_coords = self.ob_coords(buoy_pair.right)
    #         # # get the perp forward direction
    #         # forward_dir = (right_coords[1] - left_coords[1], left_coords[0] - right_coords[0])



    #         # if not passed_waypoint:
    #         if passed_previous or self.first_buoy_pair:
    #             self.follow_path_client.wait_for_server()
    #             goal_msg = FollowPath.Goal()
    #             goal_msg.planner = self.get_parameter("planner").value
    #             goal_msg.x = waypoint[0]
    #             goal_msg.y = waypoint[1]
    #             goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
    #             goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
    #             goal_msg.goal_tol = self.get_parameter("goal_tol").value
    #             goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
    #             goal_msg.choose_every = self.get_parameter("choose_every").value
    #             goal_msg.is_stationary = True
    #             self.follow_path_client.wait_for_server()
    #             self.send_goal_future = self.follow_path_client.send_goal_async(
    #                 goal_msg
    #             )
    #             self.sent_waypoints.add(waypoint)
    #             self.first_buoy_pair = False

    # def map_cb(self, msg):
    #     """
    #     When a new map is received, check if it is the first one (we haven't set up the starting buoys)
    #     and find the starting pair, and then (if the starting buoys are successfully computed) form
    #     the buoy pair / waypoint sequence
    #     """
    #     self.obstacles = msg.obstacles

    def odometry_cb(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # def execute_callback(self, goal_handle):

    #     self.start_process("Follow buoy path started!")

    #     while rclpy.ok() and self.obstacles is None:
    #         time.sleep(1.0) # TODO: maybe change this
    #         if goal_handle.is_cancel_requested:
    #             goal_handle.canceled()
    #             return Task.Result()

    #     success = False
    #     while not success:
    #         success = self.setup_buoys()
    #         time.sleep(1.0)
    #         if goal_handle.is_cancel_requested:
    #             goal_handle.canceled()
    #             return Task.Result()

    #     self.get_logger().info("Setup buoys succeeded!")

    #     while not self.result:
    #         # Check if we should abort/cancel if a new goal arrived
    #         if self.should_abort():
    #             self.end_process("New request received. Aborting path following.")
    #             goal_handle.abort()
    #             return Task.Result()

    #         if goal_handle.is_cancel_requested:
    #             self.end_process("Cancel requested. Aborting path following.")
    #             goal_handle.canceled()
    #             return Task.Result()

    #         self.generate_waypoints()

    #         time.sleep(self.timer_period)

    #     self.end_process("Follow buoy path completed!")
    #     goal_handle.succeed()
    #     return Task.Result(success=True)


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
