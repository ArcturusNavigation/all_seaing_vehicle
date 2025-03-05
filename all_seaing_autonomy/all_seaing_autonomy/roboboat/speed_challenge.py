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

TIMER_PERIOD = 1 / 60

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
            LabeledBoundingBox2D, "bounding_boxes_ycrcb", self.seg_bbox_cb, 10
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

        self.declare_parameter("xy_threshold", 2.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 5)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value
        self.declare_parameter("turn_offset", 1.0)
        self.turn_offset = self.get_parameter("turn_offset").get_parameter_value().double_value

        self.robot_pos = (0, 0)
        self.home_pos = (0, 0)
        self.blue_buoy_pos = (0, 0)
        self.runnerActivated = False
        # TODO: determine the direction of the blue buoy somewhere in the code.
        # perhaps in qualifying rounds we can enter from the correct direction, 
        # so we just take that direction?


        # unit vector in the direction of the blue buoy
        # ex: (0, -1) for south (-y), (0,1) for north (+y) 
        self.buoy_direction = (0,0) 
        self.buoy_found = False

        self.obstacles = None
        self.seg_bboxes = None

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        self.blue_labels = set()

        if self.is_sim:
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
            # hardcoded from reading YAML
            self.blue_labels.add(label_mappings["blue"])
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
            for buoy_label in ["blue_buoy", " blue_circle", "blue_racquet_ball"]:
                self.blue_labels.add(label_mappings[buoy_label])

        self.obstacles = []


    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.buoy_found = False
        self.runnerActivated = False

    
    def execute_callback(self, goal_handle):
        self.start_process("Speed challenge task started!")
            
        self.reset_challenge()
        self.get_logger().info("Speed challenge setup completed.")

        while rclpy.ok():
            # Check if the action client requested cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested. Aborting task initialization.")
                goal_handle.canceled()
                return Task.Result(success=False)
            
            if self.runnerActivated:
                self.home_pos = self.robot_pos # keep track of home position
                task_result = self.probe_blue_buoy()
                self.end_process("Speed challenge task ended.")
                return task_result


            time.sleep(self.timer_period)

        # If we exit the `while rclpy.ok()` loop somehow
        self.get_logger().info("ROS shutdown detected or loop ended unexpectedly.")
        goal_handle.abort()
        return Task.Result(success=False)

    def seg_bbox_cb(self, msg):
        '''
        Handles when an color segmented image gets published
        '''
        self.seg_bboxes = msg.boxes
        if self.runnerActivated:
            return

        ###### checks if the color segmented image depicts the LED changing from red to green
        ###### if so, make self.runnerActivated to be true.
    
    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

    def odometry_cb(self, msg):
        self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def probe_blue_buoy(self):
        '''
        Function to find the blue buoy by moving near it (general direction).
        Keeps on appending waypoints to the north/south until it finds 
        '''
        max_guide_d = 30
        guide_point = (max_guide_d*self.buoy_direction[0], max_guide_d*self.buoy_direction[1])
        self.cond_move_to_point(guide_point, self.blue_buoy_detected)
        return self.circle_blue_buoy()
    
    def circle_blue_buoy(self):
        '''
        Function to circle the blue buoy.
        '''
        if not self.blue_buoy_detected():
            self.get_logger().info("task 4 blue buoy probing exited without finding blue buoy")
            return Task(success=False)
        
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
        self.move_to_point(self.home_pos)
        return Task(success=True)

    def move_to_point(self, point):
        '''
        Moves the boat to the specified position using the follow path action server.
        Returns the future of the server request.
        '''
        self.follow_path_client.wait_for_server()
        goal_msg = FollowPath.Goal()
        goal_msg.planner = self.get_parameter("planner").value
        goal_msg.x = point.x
        goal_msg.y = point.y
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
        return self.send_goal_future

    def cond_move_to_point(self, point, cancel_cond):
        '''
        Moves the boat to the specified position using the follow path action server.
        In the middle of the following, if some condition is met, the following action is cancelled
        and the function is exited.
        '''
        future = self.move_to_point(point)
        while not future.done():
            if cancel_cond(): 
                goal_msg = FollowPath.Goal()
                goal_msg.is_cancel_requested = True
                self.follow_path_client.send_goal_async(goal_msg)

            time.sleep(TIMER_PERIOD)
        return future
    
    def blue_buoy_detected(self):
        '''
        Check if the blue buoy for turning is detected (returns boolean).
        Also sets the position of the blue buoy if it is found.
        '''
        for obstacle in self.obstacles:
            if obstacle.label in self.blue_labels: 
                # TODO: perhaps make this check better instead of just checking for a blue circle/buoy
                self.buoy_found = True
                self.blue_buoy_pos = (obstacle.global_point.point.x, obstacle.global_point.point.y)
                break

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
