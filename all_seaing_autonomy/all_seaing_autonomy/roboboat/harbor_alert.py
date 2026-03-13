#!/usr/bin/env python3
import rclpy
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor

from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import Task
from ament_index_python.packages import get_package_share_directory
from all_seaing_common.task_server_base import TaskServerBase

from all_seaing_autonomy.buoy_utils import ob_coords

import numpy as np
import os
import yaml
import time
from collections import deque
from functools import partial
from enum import Enum

class ReturnState(Enum):
    SETTING_UP = 1
    STATIONKEEPING = 2
    RETURNING = 3

class HarborAlert(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "harbor_alert_server", action_name = "harbor_alert", search_action_name = "search_harbor_alert")

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )

        self.declare_parameter("buoy_dist_thres", 40.0)
        self.buoy_dist_thres = self.get_parameter("buoy_dist_thres").get_parameter_value().double_value

        self.declare_parameter("stationkeep_dist", 2.0)
        self.stationkeep_dist = self.get_parameter("stationkeep_dist").get_parameter_value().double_value

        self.declare_parameter("probe_distance", 10)
        self.declare_parameter("adaptive_distance", 0.7)
        self.adaptive_distance = self.get_parameter("adaptive_distance").get_parameter_value().double_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

        self.declare_parameter("is_sim", False)

        self.blue_buoy_pos = np.array([0.0, 0.0])
        self.return_pos = None
        self.return_dir = None
        
        self.buoy_found = False

        self.obstacles = None

        self.state = ReturnState.SETTING_UP

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        
        self.blue_labels = set()

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


        if self.get_parameter("is_sim").get_parameter_value().bool_value:
            # hardcoded from reading YAML
            self.blue_labels.add(label_mappings["yellow"])
        else:
            for buoy_label in ["yellow_buoy", "yellow_racquet_ball"]:
                self.blue_labels.add(label_mappings[buoy_label])
    
    def should_accept_task(self, goal_request=None):
        if self.obstacles is None:
            return False
        return self.blue_buoy_detected()
    
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Task Server [{self.server_name}] received task')
        return GoalResponse.ACCEPT
    
    def search_goal_callback(self, goal_request):
        self.return_pos = np.array(self.robot_pos)
        self.return_dir = np.array(self.robot_dir)
        self.get_logger().info(f'Searching Server for [{self.server_name}] called')
        return GoalResponse.ACCEPT

    def init_setup(self):
        if self.blue_buoy_detected():
            self.state = ReturnState.STATIONKEEPING
        else:
            self.state = ReturnState.RETURNING
        self.mark_successful()

    def control_loop(self):
        if self.state == ReturnState.RETURNING:
            self.return_to_start()
            self.mark_successful()
        elif self.state == ReturnState.STATIONKEEPING:
            self.station_keep_blue_buoy()
            self.state = ReturnState.RETURNING
        

    def map_cb(self, msg):
        '''
        Gets the labeled map from all_seaing_perception.
        '''
        self.obstacles = msg.obstacles

    def station_keep_blue_buoy(self):
        self.get_logger().info("Going closer to the buoy")

        robot_pos = np.array(self.robot_pos)
        robot_buoy_vector = self.blue_buoy_pos - robot_pos
        robot_buoy_dist = np.linalg.norm(robot_buoy_vector)
        buoy_direction = robot_buoy_vector / robot_buoy_dist

        def update_current_point():
            self.blue_buoy_detected()
            return self.blue_buoy_pos - self.stationkeep_dist * buoy_direction

        self.stationkeep_point = update_current_point()
        self.move_to_point(self.stationkeep_point, busy_wait=True,
                            goal_update_func=partial(self.update_point, "stationkeep_point", self.adaptive_distance, update_current_point) )
        self.get_logger().info(f"moved to point = {self.moved_to_point}")

        return Task.Result(success=True)
    
    def return_to_start(self):
        if self.return_pos is None:
            return Task.Result(success=True)
        self.get_logger().info("Returning to pre-interruption position")
        self.move_to_point(self.return_pos, busy_wait=True)

        # if self.return_dir is None:
        #     return Task.Result(success=True)

        # self.get_logger().info('Fixing orientation')
        # self.move_to_waypoint((self.return_pos[0], self.return_pos[1], math.atan2(self.return_dir[1], self.return_dir[0])), busy_wait=True)
        
        return Task.Result(success=True)

    def blue_buoy_detected(self, buoy_front=False):
        '''
        Check if the blue buoy for turning is detected (returns boolean).
        Also sets the position of the blue buoy if it is found.
        '''
        backup_buoy = None
        updated_pos = False
        robot_pos = np.array(self.robot_pos)
        robot_dir = np.array(self.robot_dir)
        for obstacle in self.obstacles:
            if obstacle.label in self.blue_labels:
                ob_pos = ob_coords(obstacle)
                if np.linalg.norm(robot_pos - ob_pos) > self.buoy_dist_thres:
                    continue
                buoy_dir = ob_pos - robot_pos
                dot_prod = buoy_dir @ robot_dir
                if (backup_buoy is None) or (self.buoy_found and (np.linalg.norm(self.blue_buoy_pos - ob_pos) < np.linalg.norm(self.blue_buoy_pos - backup_buoy))):
                    backup_buoy = ob_pos
                if ((not buoy_front) or (dot_prod > 0)) and ((not self.buoy_found) or (np.linalg.norm(self.blue_buoy_pos - ob_pos) < self.duplicate_dist)):
                    self.buoy_found = True
                    updated_pos = True
                    self.blue_buoy_pos = ob_pos
                    break
        if (not updated_pos) and (backup_buoy is not None):
            self.get_logger().info('SWITCHING TO BACKUP BUOY')
            self.blue_buoy_pos = backup_buoy
        return self.buoy_found



def main(args=None):
    rclpy.init(args=args)
    node = HarborAlert()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
