#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_interfaces.action import Waypoint, FollowPath
from all_seaing_interfaces.msg import ControlOption
from all_seaing_navigation.a_star import AStar

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Point

TIMER_PERIOD = 1 / 60


class NavigationServer(Node):
    def __init__(self):
        super().__init__("navigation_server")

        # --------------- PARAMETERS ---------------#

        self.global_frame_id = (
            self.declare_parameter("global_frame_id", "odom")
            .get_parameter_value()
            .string_value
        )

        # --------------- SUBSCRIBERS AND PUBLISHERS ---------------#

        self.group = MutuallyExclusiveCallbackGroup()
        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self.follow_path_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")

        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )

        self.path_pub = self.create_publisher(Path, "a_star_path", 10)
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)

        # --------------- MEMBER VARIABLES ---------------#

        self.map_grid = None
        self.map_info = None
        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0
        self.proc_count = 0
        self.prev_update_time = self.get_clock().now()

    def map_callback(self, msg: OccupancyGrid):
        self.map_grid = msg.data
        self.map_info = msg.info
        self.get_logger().info("Map received")

    def odom_callback(self, msg: Odometry):
        self.nav_x = msg.pose.pose.position.x
        self.nav_y = msg.pose.pose.position.y
        _, _, self.heading = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

    def start_process(self, msg=None):
        self.proc_count += 1
        while self.proc_count >= 2:
            time.sleep(TIMER_PERIOD)
        if msg is not None:
            time.sleep(TIMER_PERIOD)
            self.get_logger().info(msg)

    def end_process(self, msg=None):
        self.delete_all_marker()
        self.proc_count -= 1
        if msg is not None:
            self.get_logger().info(msg)

    def generate_path(self, goal_handle):
        goal_tol = goal_handle.request.goal_tol
        obstacle_tol = goal_handle.request.obstacle_tol
        start = Point(x=self.nav_x, y=self.nav_y)
        goal = Point(x=goal_handle.request.x, y=goal_handle.request.y)

        if goal_handle.request.planner == FollowPath.ASTAR:
            planner = AStar(
                self.map_info, self.map_grid, start, goal, obstacle_tol, goal_tol
            )
        elif goal_handle.request.planner == FollowPath.RRT:
            raise NotImplementedError
        else:
            self.get_logger().error("Unrecognized planner name")
            raise ValueError

        path = planner.plan_path()
        path.poses = path.poses[:: goal_handle.request.choose_every]
        return path

    def send_waypoint(self, goal_handle, pose):
        goal_msg = Waypoint.Goal()
        goal_msg.xy_threshold = goal_handle.request.xy_threshold
        goal_msg.x = pose.position.x
        goal_msg.y = pose.position.y
        goal_msg.ignore_theta = True

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

    # TODO: ADD REJECT IF ASTAR PATH IS NOT FOUND
    def follow_path_callback(self, goal_handle):
        self.start_process("Path following started!")

        if self.map_grid is None:
            self.get_logger().warn(
                "OccupancyGrid has not been received! Aborting path following."
            )
            self.end_process()
            goal_handle.abort()
            return FollowPath.Result()
        path = self.generate_path(goal_handle)

        if not path.poses:
            self.get_logger().warn(
                "Planner failed to find a valid path! Aborting path following."
            )
            goal_handle.abort()
            return Waypoint.Result()

        for pose in path.poses:
            self.send_waypoint(goal_handle, pose)

            # Wait until the boat finished reaching the waypoint
            while not self.result:
                if self.proc_count >= 2:
                    self.end_process("Path following aborted!")
                    goal_handle.abort()
                    return Waypoint.Result()

                if goal_handle.is_cancel_requested:
                    self.end_process("Path following canceled!")
                    goal_handle.canceled()
                    return Waypoint.Result()

                time.sleep(TIMER_PERIOD)

        self.end_process("Path following completed!")
        goal_handle.succeed()
        return FollowPath.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
