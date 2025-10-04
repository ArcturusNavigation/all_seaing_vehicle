#!/usr/bin/env python3
from ast import Num
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle, LabeledBoundingBox2DArray, LabeledBoundingBox2D
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from tf_transformations import euler_from_quaternion

import os
import yaml
import math
import time
from collections import deque

TIMER_PERIOD = 1 / 60

class Bbox:
    def __init__(self, bbox_msg):
        self.x = bbox_msg.min_x
        self.y = bbox_msg.min_y
        self.w = bbox_msg.max_x-bbox_msg.min_x
        self.h = bbox_msg.max_y-bbox_msg.min_y

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
            LabeledBoundingBox2DArray, "bounding_boxes_ycrcb", self.seg_bbox_cb, 10
        )
        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/labeled", self.map_cb, 10
        )
        self.odometry_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odometry_cb, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_cb, 10
        )


        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("xy_threshold", 0.5)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 1)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value
        self.declare_parameter("turn_offset", 1.0)
        self.turn_offset = self.get_parameter("turn_offset").get_parameter_value().double_value

        self.robot_pos = (0, 0)
        self.robot_dir = (0, 0)
        self.home_pos = (0, 0)
        self.blue_buoy_pos = (0, 0)
        self.runnerActivated = False
        
        # NOTE: in qualifying round we assume we enter from the correct direction.


        # unit vector in the direction of the blue buoy
        # ex: (0, -1) for south (-y), (0,1) for north (+y)
        self.buoy_direction = (0,0)
        self.buoy_found = False
        self.following_guide = False

        self.obstacles = None
        self.image_size = (400,400)
        self.seg_bboxes = deque()
        self.max_seg_bboxes = 10 # guarantee this is even



        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.blue_labels = set()
        self.red_labels = set()
        self.green_labels = set()


        # TODO: change the param to be the same between is_sim and not
        # too sleepy, dont want to break things.
        # CODE IS COPIED FROM FOLLOW_BUOY_PATH,SUBJECT TO CHANGES
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
        self.red_labels.add(label_mappings["red"])
        self.green_labels.add(label_mappings["green"])


        if self.is_sim:
            # hardcoded from reading YAML
            # self.blue_labels.add(label_mappings["blue"])
            # TODO: for SIM ONLY (no blue buoy)
            self.blue_labels.add(label_mappings["red"])
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

        self.obstacles = []


    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.buoy_found = False
        self.runnerActivated = False
        self.following_guide = True

    def execute_callback(self, goal_handle):
        self.start_process("Speed challenge task started!")

        self.reset_challenge()
        self.get_logger().info("Speed challenge setup completed.")

        # station keep logic
        self.move_to_point(self.robot_pos)


        # TODO: GET RID OF THIS, FOR TESTING IN SIM ONLY
        # assumes LED has already shifted color
        self.runnerActivated = True

        while rclpy.ok():
            # Check if the action client requested cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested. Aborting task initialization.")
                goal_handle.canceled()
                return Task.Result(success=False)

            if self.runnerActivated:
                self.home_pos = self.robot_pos # keep track of home position
                self.buoy_direction = self.robot_dir
                task_result = self.probe_blue_buoy()
                self.end_process("Speed challenge task ended.")
                return task_result
                
            time.sleep(TIMER_PERIOD)

        # If we exit the `while rclpy.ok()` loop somehow
        self.get_logger().info("ROS shutdown detected or loop ended unexpectedly.")
        goal_handle.abort()
        return Task.Result(success=False)

    def seg_bbox_cb(self, msg):
        '''
        Handles when an color segmented image gets published
        '''

        '''
        For sim
        '''
        self.runnerActivated = True

        self.seg_bboxes.append(msg.boxes)
        if len(self.seg_bboxes) > self.max_seg_bboxes:
            self.seg_bboxes.popleft()
        else:
            return
        if self.runnerActivated:
            return

        all_seg_bboxes = list(self.seg_bboxes)

        redBboxes = []
        greenBboxes = []
        for frame in all_seg_bboxes: #fixed time, frame= LabeledBoundingBox2D[]
            redBboxes.append([])
            greenBboxes.append([])
            for box in frame:
                if box.label in self.red_labels:
                    redBboxes[-1].append(Bbox(box))
                if box.label in self.green_labels:
                    greenBboxes[-1].append(Bbox(box))

        cutoff = len(all_seg_bboxes)//2
        beforeRed = redBboxes[0:cutoff]
        afterRed = redBboxes[cutoff:len(redBboxes)]
        beforeGreen = greenBboxes[0:cutoff]
        afterGreen = greenBboxes[cutoff:len(greenBboxes)]

        # TODO: FIX MAGIC NUMBER TERRITORY
        epsilon = 0.02*math.sqrt(self.image_size[0]**2 + self.image_size[1]**2)
        lmbda = epsilon
        p = 5
        limit = 0.5

        if self.led_changed(beforeRed, afterRed, beforeGreen, afterGreen, epsilon, lmbda, p, limit):
            self.get_logger().info("Detected LED color change!")
            self.runnerActivated = True

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

    def odometry_cb(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        (row, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_dir = (math.cos(yaw), math.sin(yaw))

    def camera_info_cb(self, msg):
        '''
        Gets camera image info from all_seaing_perception.
        '''
        self.image_size = (msg.width, msg.height)

    def probe_blue_buoy(self):
        '''
        Function to find the blue buoy by moving near it (general direction).
        Keeps on appending waypoints to the north/south until it finds 
        '''
        self.get_logger().info("Probing for blue buoy")
        max_guide_d = 30
        guide_point = (max_guide_d*self.buoy_direction[0] + self.robot_pos[0], 
                        max_guide_d*self.buoy_direction[1] + self.robot_pos[1])
        self.get_logger().info(f"Current position: {self.robot_pos}. Guide point: {guide_point}.")

        future = self.move_to_point(guide_point)
        future.add_done_callback(self.future_done)
        while self.following_guide:
            if self.blue_buoy_detected():
                self.move_to_point(self.robot_pos,is_stationary=False)
                break
            time.sleep(TIMER_PERIOD)
        return self.circle_blue_buoy()

    def future_done(self, future):
        '''
        Set self.following_guide to False when the future is done.
        '''
        self.following_guide = False
        pass

    def circle_blue_buoy(self):
        '''
        Function to circle the blue buoy.
        '''
        self.get_logger().info("Circling blue buoy")
        if not self.blue_buoy_detected():
            self.get_logger().info("task 4 blue buoy probing exited without finding blue buoy")
            return Task.Result(success=False)
        
        #circle the blue buoy like a baseball diamond
        # a better way to do this might be to have the astar run to original cell, 
        # but require the path to go around buoy

        t_o = self.turn_offset
        first_dir = (self.buoy_direction[1]*t_o, -self.buoy_direction[0]*t_o)
        second_dir = (self.buoy_direction[0]*t_o, self.buoy_direction[1]*t_o)
        third_dir = (-first_dir[0]*t_o, -first_dir[1]*t_o)
        
        first_base = self.add_tuple(self.blue_buoy_pos, first_dir)
        second_base = self.add_tuple(self.blue_buoy_pos, second_dir)
        third_base = self.add_tuple(self.blue_buoy_pos, third_dir)
        
        self.move_to_point(first_base)
        self.move_to_point(second_base)
        self.move_to_point(third_base)

        return self.return_to_start()
    
    def return_to_start(self):
        '''
        After circling the buoy, return to the starting position.
        '''
        self.get_logger().info("Returning to start")
        self.move_to_point(self.home_pos)
        return Task.result(success=True)

    # robust realtime visual signal processing
    def led_changed(self, beforeRed, afterRed, beforeGreen, afterGreen, epsilon, lmbda, p, limit):
        '''
        beforeRed :: [[Bbox]] | len(beforeRed) > 0
        frames with red bounding boxes before signal event

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
        fmap = lambda f, l: [f(x) for x in l]  # A → B → [A] → [B]
        flatten = lambda l: l[0] + flatten(l[1:]) if len(l) > 0 else []  # [[A]] → [A]
        filter = lambda p, l: [x for x in l if p(x)]  # (A → Bool) → [A] → [A]
        product = lambda a, b: flatten([[(alpha, beta) for alpha in a] for beta in b])  # [A] → [B] → [(A, B)]
        access = lambda l1, l2: [l1[i] for i in l2]  # [A] → [Int] → [A]
        distinguishable = lambda dist: dist > epsilon  # Num → Bool
        matching = lambda dist: dist < lmbda  # Num → Bool
        changed = lambda prob: prob >= limit  # Prob → Bool
        multiply = lambda x: x[0] * x[1]  # (Num, Num) → Num
        norm = lambda b1: lambda b2: math.sqrt((b1.x - b2.x) ** 2 + (b1.y - b2.y) ** 2)  # Num → (Num → Num)
        estimator = lambda trials: sum(trials) / len(trials)  # [Bool] → Num

        # [Bbox] → (Bbox → Bool)
        # whether the bounding box is distinguishable from all bounding boxes in a list of bounding boxes
        distinct = lambda boxes: lambda box: all(fmap(distinguishable, fmap(norm(box), boxes)))

        # Int | nBefore > 0
        # number of "before" frames
        nBefore = len(beforeRed)

        # Int | nAfter > 0
        # number of "after" frames
        nAfter = len(afterGreen)

        # Int → Int → Int | n > 0 → [Int]
        # generate `n` evenly spaced elements from an arbitrary discrete range
        linspace = lambda a, b, n: [a] + linspace(a + (b-a)//(n-1), b, n-1) if n > 1 else [a]

        # [Int]
        # indices of "before" frames to sample
        beforeSampleIndices = fmap(int, linspace(0, nBefore-1, p))

        # ([Bbox], [Bbox]) → [Bbox]
        # remove first bounding boxes that are indistinguishable from second ones in the same frame
        filterFrame = lambda colors: filter(distinct(colors[1]), colors[0])

        # [[Bbox]] → [[Bbox]]
        # remove red bounding boxes that are indistinguishable from green ones for each frame
        beforeRedCandidates = fmap(filterFrame, zip(beforeRed, beforeGreen))

        # [[Bbox]]
        # bounding box data for red before frames
        beforeSampleRed = access(beforeRedCandidates, beforeSampleIndices)

        # [Bbox]
        # list of identified red bounding boxes from sample frames
        redBboxes = flatten(beforeSampleRed)

        # Bbox → ([Bbox] → Bool)
        # detect whether a particular bounding box exists in a different frame
        bboxExists = lambda bbox: lambda frame: any(fmap(matching, fmap(norm(bbox), frame)))

        # Bbox → [[Bbox]] → Prob
        # determine probability that a given bounding box exists in time series
        probExistence = lambda box, series: estimator(fmap(bboxExists(box), series))

        # [Int]
        # indices of "after" frames to sample
        afterSampleIndices = fmap(int, linspace(0, nAfter-1, p))

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
        probabilities = fmap(candidateProbability, signalCandidates)

        # [Prob]
        # probability of existence of each bounding box pair in signal candidates
        pairProbabilities = fmap(multiply, probabilities)

        # check for any probability exceeding confidence threshold
        return any(fmap(changed, pairProbabilities))

    def move_to_point(self, point, is_stationary=True):
        '''
        Moves the boat to the specified position using the follow path action server.
        Returns the future of the server request.
        '''
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
        return self.send_goal_future

    def blue_buoy_detected(self):
        '''
        Check if the blue buoy for turning is detected (returns boolean).
        Also sets the position of the blue buoy if it is found.
        '''
        for obstacle in self.obstacles:
            if obstacle.label in self.blue_labels:
                # TODO: perhaps make this check better instead of just checking for a blue circle/buoy
                self.get_logger().info("Found blue buoy.")
                self.buoy_found = True
                self.blue_buoy_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
                break
        self.get_logger().info(f"blue buoy found: {self.buoy_found}.") 
        return self.buoy_found

    def add_tuple(self, a, b):
        '''
        function to add two tuples
        why is this here
        '''
        return tuple(sum(x) for x in zip(a, b))

def main(args=None):
    rclpy.init(args=args)
    node = SpeedChange()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
