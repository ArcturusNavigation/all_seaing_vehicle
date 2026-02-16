#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from all_seaing_interfaces.action import Task, Search
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.srv import RestartSLAM
from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_common.report_pb2 import RobotState, LatLng, TaskType, SoundSignal, SignalType
import all_seaing_common.report_pb2
from all_seaing_interfaces.msg import Heartbeat
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math
from enum import Enum

from sensor_msgs.msg import Joy
from action_msgs.msg import GoalStatus
from dataclasses import dataclass
import time

###

# test using:
# ros2 topic pub -1 /boat_info all_seaing_interfaces/msg/BoatInfo "{last_seen: {sec: 0, nanosec: 0}, label: 1, x_pos: 100, y_pos: 200}"

###

class ReferenceInt:
    def __init__(self, val: int):
        self.val = val

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

class ActionType(Enum):
    TASK = 1
    SEARCH = 2

@dataclass
class RestartSLAMOptions:
    restart_position: bool = False
    restart_buoys: bool = False
    restart_banners: bool = False

class RunTasks(ActionServerBase):
    def __init__(self):
        super().__init__("run_tasks")
        self.init_tasks = [
            ActionClient(self, Task, "task_init")
        ]
        self.task_list = [
            # ENTRY GATES
            # [ActionType.SEARCH, TaskType.TASK_ENTRY_EXIT, ActionClient(self, Search, "search_entry"), ReferenceInt(0), ReferenceInt(0), "entry"],
            # [ActionType.TASK, TaskType.TASK_ENTRY_EXIT, ActionClient(self, Task, "entry_gates"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions()],

            # FOLLOW PATH
            [ActionType.SEARCH, TaskType.TASK_NAV_CHANNEL, ActionClient(self, Search, "search_followpath"), ReferenceInt(0), ReferenceInt(0), "follow_path"],
            [ActionType.TASK, TaskType.TASK_NAV_CHANNEL, ActionClient(self, Task, "follow_buoy_path"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions(True, True, False)],

            # SPEED CHALLENGE
            # [ActionType.SEARCH, TaskType.TASK_SPEED_CHALLENGE, ActionClient(self, Search, "search_speed"), ReferenceInt(0), ReferenceInt(0), "speed_challenge"],
            # [ActionType.TASK, TaskType.TASK_SPEED_CHALLENGE, ActionClient(self, Task, "speed_challenge"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions(True, True, False)],

            # DOCKING
            # [ActionType.SEARCH, TaskType.TASK_DOCKING, ActionClient(self, Search, "search_docking"), ReferenceInt(0), ReferenceInt(0), "docking"],
            # [ActionType.TASK, TaskType.TASK_DOCKING, ActionClient(self, Task, "docking"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions(True, True, False)],

            # DELIVERY
            # [ActionType.SEARCH, TaskType.TASK_OBJECT_DELIVERY, ActionClient(self, Search, "search_delivery"), ReferenceInt(0), ReferenceInt(0), "delivery"],
            # [ActionType.TASK, TaskType.TASK_OBJECT_DELIVERY, ActionClient(self, Task, "mechanism_navigation"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions(True, True, False)],

            # EXIT GATES
            # [ActionType.SEARCH, TaskType.TASK_ENTRY_EXIT, ActionClient(self, Search, "search_return"), ReferenceInt(0), ReferenceInt(0), "return"],
            # [ActionType.TASK, TaskType.TASK_ENTRY_EXIT, ActionClient(self, Task, "return_home"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions(True, True, False)],
            
            # FALLBACKS
            # [ActionType.TASK, TaskType.TASK_UNKNOWN, ActionClient(self, Task, "follow_buoy_pid"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions()],
            # [ActionType.TASK, TaskType.TASK_SPEED_CHALLENGE, ActionClient(self, Task, "speed_challenge_pid"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions()],
            # [ActionType.TASK, TaskType.TASK_DOCKING, ActionClient(self, Task, "docking_fallback"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions()],
            # [ActionType.TASK, TaskType.TASK_OBJECT_DELIVERY, ActionClient(self, Task, "delivery_qual"), ReferenceInt(0), ReferenceInt(0), None, RestartSLAMOptions()],
        ]

        self.term_tasks = [
            # RETURN TO HOME
            # [ActionType.SEARCH, ActionClient(self, Search, "search_return"), "return"],
            # [ActionType.TASK, ActionClient(self, Task, "return_home")],
        ]

        self.harbor_alert_tasks = [
            # HARBOR ALERT
            [ActionType.SEARCH, ActionClient(self, Search, "search_harbor_alert"), "harbor_sprint", "harbor_marina"],
            [ActionType.TASK, ActionClient(self, Task, "harbor_alert"), RestartSLAMOptions(True, True, False)],
        ]

        self.current_task = None
        self.next_task_index = -1
        self.next_init_index = ReferenceInt(0)
        self.next_term_index = ReferenceInt(0)
        self.next_harbor_index = ReferenceInt(0)
        self.declare_parameter("max_attempt_count", 1)
        self.max_attempt_count = self.get_parameter("max_attempt_count").get_parameter_value().integer_value

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter("red_left", True)
        self.red_left = self.get_parameter("red_left").get_parameter_value().bool_value
        
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

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.restart_slam_client = self.create_client(RestartSLAM, "restart_slam")

        self.pause_publisher = self.create_publisher(Bool, "pause", 10)
        self.find_task() # um idk if this is right

        self.obstacles = None

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
            self.green_labels.add(label_mappings["green_buoy"])
            self.green_labels.add(label_mappings["green_circle"])
            self.green_labels.add(label_mappings["green_pole_buoy"])
            self.red_labels.add(label_mappings["red_buoy"])
            self.red_labels.add(label_mappings["red_circle"])
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

        task_location_mappings_file = self.get_parameter(
            "task_locations_file"
        ).value
        with open(task_location_mappings_file, "r") as f:
            self.task_location_mappings = yaml.safe_load(f)

        with open(self.get_parameter("latlng_locations_file").value, "r") as f:
            self.latlng_location_mappings = yaml.safe_load(f)

        self.declare_parameter("location", "nbpark")
        self.location = self.get_parameter("location").get_parameter_value().string_value

        self.latlng_origin = self.latlng_location_mappings[self.location]
        
        self.gate_pair = None

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        # harbor alert

        self.keyboard_sub = None
        # if self.is_sim: 
        #     self.get_logger().info("Running in simulation mode. Listening to joystick input.")
        #     self.keyboard_sub = self.create_subscription(
        #         Joy, "/joy", self.sim_keyboard_callback, 10
        #     )
        # else: 
        self.get_logger().info("Running in real mode. Listening to keyboard input.")
        self.keyboard_sub = self.create_subscription(
            String, "/harbor_detect", self.real_harbor_callback, 10
        )
        self.harbor_alerted = False
        self.harbor_index = 0
        self.cancelled_task_harbor = True

        self.result_future = None
        self.goal_handle = None


        # Roboboat comms protocol (shore heartbeat)
        self.vel = 0
        self.odom_sub = self.create_subscription(
            Odometry, "odometry/gps", self.odom_cb, 10
        )
        
        self.heartbeat_sub = self.create_subscription(Heartbeat, "heartbeat", self.receive_heartbeat, 10)
        self.heartbeat_state = RobotState.STATE_AUTO
        self.shore_heartbeat_reporter = self.create_timer(1, self.report_shore_heartbeat)

        self.restart_slam_options = None

    def odom_cb(self, msg):
        self.vel = self.norm((msg.twist.twist.linear.x, msg.twist.twist.linear.y))

    def receive_heartbeat(self, msg):
        self.heartbeat_msg = msg
        if msg.e_stopped:
            self.heartbeat_state = RobotState.STATE_KILLED
        else:
            if msg.in_teleop:
                self.heartbeat_state = RobotState.STATE_MANUAL
            else:
                self.heartbeat_state = RobotState.STATE_AUTO

    def map_cb(self, msg):
        self.obstacles = msg.obstacles

        if self.gate_pair is None:
            self.setup_buoys()

    def report_shore_heartbeat(self):
        RAD_TO_DEG = 180.0 / math.pi
        current_task = TaskType.TASK_NONE
        if self.current_task_type != None:
            current_task = self.current_task_type

        self.report_data(all_seaing_common.report_pb2.Heartbeat(
                            state=self.heartbeat_state,
                            position=self.pos_to_latlng(self.latlng_origin, self.robot_pos), # (east, north)
                            spd_mps=self.vel,
                            heading_deg= ((90 - (RAD_TO_DEG) * (self.get_robot_pose()[2])) % 360), # (east, north, heading), deal with CW / CCW
                            current_task=current_task))
        
    def sim_keyboard_callback(self, msg):
        if self.harbor_alerted:
            return
        if msg.buttons[3]:
            self.get_logger().info(f'HARBOR ALERTED')
            self.harbor_alerted = True
            self.harbor_index = 0
            self.cancel_current_task()
            self.find_task()
    
    def real_harbor_callback(self, msg):
        if self.harbor_alerted:
            return
        freq, type = msg.data.split("_")
        if type == "single":
            self.get_logger().info(f'HARBOR ALERTED SINGLE AT {freq}HZ')
            self.harbor_alerted = True
            self.harbor_index = 0
            self.report_data(SoundSignal(
                signal_type=SignalType.SIGNAL_ONE_BLAST,
                frequency_hz=int(freq),
                assigned_task=TaskType.TASK_SPEED_CHALLENGE,
            ))
            self.cancel_current_task()
            self.find_task()
        elif type == "double":
            self.get_logger().info(f'HARBOR ALERTED DOUBLE AT {freq}HZ')
            self.harbor_alerted = True
            self.harbor_index = 1
            self.report_data(SoundSignal(
                signal_type=SignalType.SIGNAL_TWO_BLAST,
                frequency_hz=int(freq),
                assigned_task=TaskType.TASK_DOCKING,
            ))
            self.cancel_current_task()
            self.find_task()
    
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

    def ob_coords(self, buoy, local=False):
        if local:
            return (buoy.local_point.point.x, buoy.local_point.point.y)
        else:
            return (buoy.global_point.point.x, buoy.global_point.point.y)
        
    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)
    
    def obs_to_pos_label(self, obs):
        return [self.ob_coords(ob, local=False) + (ob.label,) for ob in obs]
    
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
            if ob.local_point.point.x > 0 and self.norm(self.robot_pos, self.ob_coords(ob)) < self.gate_dist_thres        
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
        self.get_logger().info(f'FOUND STARTING GATE')
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        self.gate_mid, self.gate_dir = self.midpoint_pair_dir(self.gate_pair, 0.0)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(self.gate_mid), 0.0)]))
        return True
    
    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))

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
    
    def map_cb(self, msg):
        self.obstacles = msg.obstacles

        if self.gate_pair is None:
            self.setup_buoys()

    def attempt_task(self, action_type, task_type, current_task, incr_on_success, incr_on_fail = None, location_name = None, restart_slam_options = None):
        self.current_task = current_task
        self.current_task_type = task_type
        self.incr_on_success = incr_on_success
        self.incr_on_fail = incr_on_fail
        self.get_logger().info("Starting Task Manager States...")
        self.current_task.wait_for_server()
        self.get_logger().info(f"Starting task: {self.current_task}")
        if action_type == ActionType.TASK:
            task_goal_msg = Task.Goal()
        else:
            assert location_name is not None, f"Didn't provide location to search for task {self.current_task}"
            local_x, local_y = self.task_location_mappings[location_name]["x"], self.task_location_mappings[location_name]["y"]
            if self.gate_pair is None:
                self.get_logger().info(f"Didn't identify the starting gate yet, but called to search task")
                return
            task_x = self.gate_mid[0] + self.gate_dir[1]*local_x + self.gate_dir[0]*local_y
            task_y = self.gate_mid[1] + self.gate_dir[1]*local_y - self.gate_dir[0]*local_x
            task_theta = 0.0
            search_theta = self.task_location_mappings[location_name]["search_theta"]
            if search_theta:
                task_theta = math.atan2(self.gate_dir[1], self.gate_dir[0]) + self.task_location_mappings[location_name]["theta"] - math.pi/2.0
            task_goal_msg = Search.Goal(x = task_x, y = task_y, include_theta = search_theta, theta = task_theta, wait_time = self.task_location_mappings[location_name]["wait_time"])
        self.get_logger().info(f"Sending goal: {task_goal_msg}")
        self.restart_slam_options = restart_slam_options
        send_goal_future = self.current_task.send_goal_async(
            task_goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_current_task(self):
        self.get_logger().info('TRYING TO CANCEL CURRENT TASK')
        if self.result_future is None or self.goal_handle is None: # TODO how to handle waypoint sent (& goal handle received) after us trying to cancel it? maybe ROS delay + retry for 3 times or smth
            self.get_logger().info('FUTURE IS NONE')
            self.cancelled_task_harbor = True
            return
        if ((self.result_future.result() is None) or 
            (self.result_future.result().status not in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING])):
            self.goal_handle.cancel_goal_async()
            self.cancelled_task_harbor = False
            self.get_logger().info('CANCELLED TASK')

    def find_task(self):
        if self.next_init_index.val < len(self.init_tasks):
            self.attempt_task(ActionType.TASK, TaskType.TASK_NONE, self.init_tasks[self.next_init_index.val], self.next_init_index, None)
            self.harbor_alerted = False
            return
        
        if self.harbor_alerted:
            if self.next_harbor_index.val >= len(self.harbor_alert_tasks):
                self.harbor_alerted = False
                self.next_harbor_index.val = 0
            else:
                self.attempt_task(self.harbor_alert_tasks[self.next_harbor_index.val][0], TaskType.TASK_SOUND_SIGNAL, self.harbor_alert_tasks[self.next_harbor_index.val][1], self.next_harbor_index, None, self.harbor_alert_tasks[self.next_harbor_index.val][2+self.harbor_index] if self.harbor_alert_tasks[self.next_harbor_index.val][0] == ActionType.SEARCH else None, self.harbor_alert_tasks[self.next_harbor_index.val][2])
                return
        # print(self.task_list)

        for _ in range(len(self.task_list)):
            self.next_task_index += 1
            if self.next_task_index >= len(self.task_list):
                self.next_task_index -= len(self.task_list)
            if self.task_list[self.next_task_index][3].val == 0 and self.task_list[self.next_task_index][4].val < self.max_attempt_count:
                self.attempt_task(*self.task_list[self.next_task_index])
                return
        
        if self.next_term_index.val < len(self.term_tasks):
            self.attempt_task(self.term_tasks[self.next_term_index.val][0], TaskType.TASK_NONE, self.term_tasks[self.next_term_index.val][1], self.next_term_index, None, self.term_tasks[self.next_term_index.val][2] if self.term_tasks[self.next_term_index.val][0] == ActionType.SEARCH else None)
            return
        
        self.get_logger().info("All tasks completed. Shutting down node.")
        self.destroy_node()
        return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
        self.get_logger().info(f"Current task: {self.current_task}")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected, looking for new task")
            if not (self.incr_on_fail is None): # consider failure to start task (goal rejected when setup conditions not met) as failure to do the task?
                self.incr_on_fail.val += 1
            self.find_task()
            return
        self.get_logger().info("Goal accepted")
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Task success, marked as completed, looking for new task")
            self.incr_on_success.val += 1
        else:
            if not self.cancelled_task_harbor:
                self.cancelled_task_harbor = True
                return
            self.get_logger().info("Task failure, looking for new task")
            if not (self.incr_on_fail is None):
                self.incr_on_fail.val += 1
        if (self.restart_slam_options is not None) and self.restart_slam_client.wait_for_service(2.0): # self.restart_slam_client.service_is_ready()
            self.get_logger().info(f"Calling restart SLAM service w/ options: {self.restart_slam_options}")
            restart_future = self.restart_slam_client.call_async(RestartSLAM.Request(restart_position=self.restart_slam_options.restart_position,
                                                                    restart_buoys=self.restart_slam_options.restart_buoys,
                                                                    restart_banners=self.restart_slam_options.restart_banners))
            
            time.sleep(1.0)
            # # Spin until the future is complete with a timeout
            # timeout_start = time.time()
            # while rclpy.ok():
            #     if restart_future.done():
            #         try:
            #             response = restart_future.result()
            #             self.get_logger().error(f'Service call succeeded')
            #             break
            #         except Exception as e:
            #             self.get_logger().error(f'Service call failed: {e}')
            #             break
                
            #     if time.time() - timeout_start > 2.0:
            #         self.get_logger().error('Service call timed out')
            #         break
                
            #     # Spin a little to process the response if it arrives
            #     rclpy.spin_once(self)
        # self.get_logger().error('Exiting result callback')
        self.restart_slam_options = None
        self.find_task()

def main(args=None):
    rclpy.init(args=args)
    node = RunTasks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()






