#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool

from all_seaing_interfaces.action import Task, Search
from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_common.action_server_base import ActionServerBase
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import cos, sin
from tf_transformations import euler_from_quaternion

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

class RunTasks(ActionServerBase):
    def __init__(self):
        super().__init__("run_tasks")
        self.init_tasks = [
            ActionClient(self, Task, "task_init")
        ]
        self.task_list = [
            [ActionClient(self, Task, "follow_buoy_path"), ReferenceInt(0), ReferenceInt(0), ActionClient(self, Search, "search_followpath"), "follow_path"],
            # [ActionClient(self, Task, "speed_challenge"), ReferenceInt(0), ReferenceInt(0), ActionClient(self, Search, "search_speed"), "speed_challenge"],
            # [ActionClient(self, Task, "docking"), ReferenceInt(0), ReferenceInt(0), ActionClient(self, Search, "search_docking"), "docking"],
            # [ActionClient(self, Task, "mechanism_navigation"), ReferenceInt(0), ReferenceInt(0), ActionClient(self, Search, "search_delivery"), "delivery"],
            # [ActionClient(self, Task, "follow_buoy_pid"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "speed_challenge_pid"), ReferenceInt(0), ReferenceInt(0)],
            # [ActionClient(self, Task, "docking_fallback"), ReferenceInt(0), ReferenceInt(0)],
        ]
        self.term_tasks = [
            
        ]
        self.current_task = None
        self.next_task_index = -1
        self.next_init_index = ReferenceInt(0)
        self.next_term_index = ReferenceInt(0)
        self.declare_parameter("max_attempt_count", 1)
        self.max_attempt_count = self.get_parameter("max_attempt_count").get_parameter_value().integer_value

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value
        
        self.declare_parameter("gate_dist_thres", 25.0)
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
        task_location_mappings_file
        self.declare_parameter(
            "task_locations_file", 
            os.path.join(
                bringup_prefix, "config", "course", "task_locations.yaml"
            ),
        )

        task_location_mappings_file = self.get_parameter(
            "task_locations_file"
        ).value
        with open(task_location_mappings_file, "r") as f:
            task_location_mappings = yaml.safe_load(f)
        
        self.red_left = True
        self.gate_pair = None

        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )
    
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
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            # if ((pointing_direction is None and ob.local_point.point.x > 0) or (pointing_direction is not None and self.dot(self.difference(self.robot_pos, self.ob_coords(ob)), pointing_direction) > 0)) and self.norm(self.robot_pos, self.ob_coords(ob)) < self.gate_dist_thres
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
        self.get_logger().info(f'FOUND STARTING GATE')
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        gate_wpt, _ = self.midpoint_pair_dir(self.gate_pair, 0.0)
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.gate_pair.left, self.gate_pair.right, self.pair_to_pose(gate_wpt), 0.0)]))
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

    def attempt_task(self, current_task, incr_on_success, incr_on_fail = None, search_client = None, location_name = None):
        self.current_task = current_task
        self.incr_on_success = incr_on_success
        self.incr_on_fail = incr_on_fail
        self.get_logger().info("Starting Task Manager States...")
        self.current_task.wait_for_server()
        self.get_logger().info(f"Starting task: {self.current_task}")
        task_goal_msg = Task.Goal()
        self.get_logger().info(f"Sending goal: {task_goal_msg}")
        send_goal_future = self.current_task.send_goal_async(
            task_goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def find_task(self):
        if self.next_init_index.val < len(self.init_tasks):
            self.attempt_task(self.init_tasks[self.next_init_index.val], self.next_init_index, None)
            return

        # print(self.task_list)

        for _ in range(len(self.task_list)):
            self.next_task_index += 1
            if self.next_task_index >= len(self.task_list):
                self.next_task_index -= len(self.task_list)
            if self.task_list[self.next_task_index][1].val == 0 and self.task_list[self.next_task_index][2].val < self.max_attempt_count:
                self.attempt_task(*self.task_list[self.next_task_index])
                return
        
        if self.next_term_index.val < len(self.term_tasks):
            self.attempt_task(self.term_tasks[self.next_term_index.val], self.next_term_index, None)
            return
        
        self.get_logger().info("All tasks completed. Shutting down node.")
        self.destroy_node()
        return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.time_elapsed:.2f} seconds elapsed")
        self.get_logger().info(f"Current task: {self.current_task}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected, looking for new task")
            if not (self.incr_on_fail is None): # consider failure to start task (goal rejected when setup conditions not met) as failure to do the task?
                self.incr_on_fail.val += 1
            self.find_task()
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Task success, marked as completed, looking for new task")
            self.incr_on_success.val += 1
        else:
            self.get_logger().info("Task failure, looking for new task")
            if not (self.incr_on_fail is None):
                self.incr_on_fail.val += 1
        self.find_task()

def main(args=None):
    rclpy.init(args=args)
    node = RunTasks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()






