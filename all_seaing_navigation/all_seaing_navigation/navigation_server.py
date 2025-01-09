#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_interfaces.action import Waypoint, FollowPath
from all_seaing_navigation.a_star import AStar

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry, OccupancyGrid
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseArray

TIMER_PERIOD = 1 / 60
MARKER_NS = "navigation"


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

        self.marker_pub = self.create_publisher(Marker, "action_marker", 10)

        # --------------- MEMBER VARIABLES ---------------#

        self.map = None
        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0
        self.proc_count = 0
        self.prev_update_time = self.get_clock().now()

    def cancel_callback(self, cancel_request):
        return CancelResponse.ACCEPT

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

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

    def delete_all_marker(self):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = MARKER_NS
        marker_msg.action = Marker.DELETEALL
        self.marker_pub.publish(marker_msg)

    def visualize_path(self, path: PoseArray):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_idc
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = MARKER_NS
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.05
        marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker_msg.points = [
            Point(x=pose.position.x, y=pose.position.y) for pose in path.poses
        ]
        self.marker_pub.publish(marker_msg)

    def generate_path(self, goal_handle):
        goal_tol = goal_handle.request.goal_tol
        obstacle_tol = goal_handle.request.obstacle_tol
        start = Point(x=self.nav_x, y=self.nav_y)
        goal = Point(x=goal_handle.request.x, y=goal_handle.request.y)

        if goal_handle.request.planner == "astar":
            planner = AStar(
                self.map, start, goal, obstacle_tol, goal_tol
            )
        elif goal_handle.request.planner == "rrt":
            raise NotImplementedError
        else:
            self.get_logger().error("Unrecognized planner name")
            raise ValueError

        path = planner.plan()
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

    def follow_path_callback(self, goal_handle):
        self.start_process("Path following started!")

        if self.map is None:
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
            self.end_process()
            goal_handle.abort()
            return FollowPath.Result()

        self.visualize_path(path)

        for pose in path.poses:
            self.send_waypoint(goal_handle, pose)

            # Wait until the boat finished reaching the waypoint
            while not self.result:
                if self.proc_count >= 2:
                    self.end_process("Path following aborted!")
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
