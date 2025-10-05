#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_interfaces.action import Waypoint, FollowPath
from all_seaing_navigation.planner_executor import PlannerExecutor

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray

from threading import Semaphore, Event
import math
import time

TIMER_PERIOD = 1 / 60

class NavigationServer(ActionServerBase):
    def __init__(self):
        super().__init__("navigation_server")

        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self.follow_path_callback,
            cancel_callback=self.default_cancel_callback,
            #TODO: refactor cancel logic for navigation
        )
        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/dynamic_map", self.map_callback, 10
        )

        self.map = None

        self.stop_plan_semaphore = Semaphore(1)
        self.stop_plan_evt = Event()

    def start_plan(self):
        self.stop_plan_evt.set()
        self.stop_plan_semaphore.acquire()
        self.stop_plan_evt.clear()

    def should_abort_plan(self):
        return self.stop_plan_evt.is_set()

    def stopped_plan(self):
        self.stop_plan_semaphore.release()

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def visualize_path(self, path: PoseArray):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = self.marker_ns
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.1
        marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker_msg.points = [
            Point(x=pose.position.x, y=pose.position.y) for pose in path.poses
        ]
        self.marker_pub.publish(marker_msg)

    def generate_path(self, goal_handle, nav_x, nav_y):
        self.start_plan()  # Protect the long-running generate path function with semaphores

        start = Point(x=nav_x, y=nav_y)
        goal = Point(x=goal_handle.request.x, y=goal_handle.request.y)
        obstacle_tol = goal_handle.request.obstacle_tol
        goal_tol = goal_handle.request.goal_tol

        self.planner = PlannerExecutor(goal_handle.request.planner)
        path = self.planner.plan(self.map, start, goal, obstacle_tol, goal_tol, self.should_abort_plan)
        path.poses = path.poses[(len(path.poses) - 1) % goal_handle.request.choose_every :: goal_handle.request.choose_every]

        self.stopped_plan()  # Release the semaphore
        return path

    def send_waypoint(self, goal_handle, pose, is_stationary):
        goal_msg = Waypoint.Goal()
        goal_msg.xy_threshold = goal_handle.request.xy_threshold
        goal_msg.theta_threshold = goal_handle.request.theta_threshold
        goal_msg.x = pose.position.x
        goal_msg.y = pose.position.y
        goal_msg.ignore_theta = True
        goal_msg.is_stationary = is_stationary
        self.result = False
        self.waypoint_client.wait_for_server()
        self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.waypoint_response_callback)

    def waypoint_response_callback(self, future):
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.waypoint_result_callback)

    def waypoint_result_callback(self, future):
        self.result = future.result().result.is_finished

    def find_first_waypoint(self, path, lookahead, nav_x, nav_y):
        last_in_lookahead = -1
        closest_wpt = 0
        min_dist = math.inf
        for i, pose in enumerate(path.poses):
            dx = pose.position.x - nav_x
            dy = pose.position.y - nav_y
            dist = math.hypot(dx, dy)
            if dist < lookahead:
                last_in_lookahead = i
            if dist < min_dist:
                closest_wpt = i
                min_dist = dist
        return closest_wpt if last_in_lookahead == -1 else last_in_lookahead + 1

    def follow_path_callback(self, goal_handle):
        self.get_logger().info("Path following started!")

         # Immediately start and end process if no map is found
        if self.map is None:
            self.get_logger().info("No valid path found. Aborting path following.")
            goal_handle.abort()
            return FollowPath.Result()

        # Get robot pose
        nav_x, nav_y, _ = self.get_robot_pose()

        # Generate path using requested planner
        path = self.generate_path(goal_handle, nav_x, nav_y)
        if not path.poses:
            self.get_logger().info("No valid path found. Aborting path following.")
            goal_handle.abort()
            return FollowPath.Result()

        self.start_process()

        self.visualize_path(path)

        first_wpt_idx = self.find_first_waypoint(path, goal_handle.request.xy_threshold, nav_x, nav_y)
        for i, pose in enumerate(path.poses):
            # Skip if waypoints before first_wpt_idx (boat shouldn't backtrack)
            if i < first_wpt_idx:
                continue

            # Last request should be "station keeping" if is_stationary is True
            is_stationary = (i == len(path.poses)-1 and goal_handle.request.is_stationary)

            self.send_waypoint(goal_handle, pose, is_stationary)

            # Wait until the boat finished reaching the waypoint
            while not self.result:
                if self.should_abort():
                    self.end_process("New request received. Aborting path following.")
                    goal_handle.abort()
                    return FollowPath.Result()

                if goal_handle.is_cancel_requested:
                    self.end_process("Path following canceled!")
                    goal_handle.canceled()
                    return FollowPath.Result()

                time.sleep(TIMER_PERIOD)

        self.end_process("Path following completed!")
        goal_handle.succeed()
        return FollowPath.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
