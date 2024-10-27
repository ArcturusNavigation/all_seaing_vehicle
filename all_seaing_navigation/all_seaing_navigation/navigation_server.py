#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time

from all_seaing_interfaces.action import FollowPath
from all_seaing_interfaces.msg import ControlMessage
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker


from all_seaing_interfaces.action import FollowPath

class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            self.execute_callback)

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            execute_callback=self.waypoint_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.group,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )


    #TODO: create a subscription to take in the pose array from a_star.py


    #TODO: let the robot follow a path (series of points) instead of 1
    def send_waypoint(self, x, y, angular):
        control_msg = ControlMessage()
        control_msg.priority = 1
        control_msg.linear_control_mode = ControlMessage.WORLD_POSITION
        control_msg.angular_control_mode = ControlMessage.WORLD_POSITION
        control_msg.x = x
        control_msg.y = y
        control_msg.angular = angular
        self.control_pub.publish(control_msg)

    #TODO: return result
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        return FollowPath.Result(is_finished=True)

    def cancel_callback(self, cancel_request):
        return CancelResponse.ACCEPT

    #TODO: change everything to send navigation server-related messages?
    def navigation_callback(self, goal_handle):
        self.start_process("Navigation following started!")

        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_theta = (
            math.atan2(goal_y - self.nav_y, goal_x - self.nav_x)
            if goal_handle.request.ignore_theta
            else goal_handle.request.theta
        )
        self.visualize_waypoint(goal_x, goal_y)

        feedback_msg = FollowPath.Feedback()
        while (
            abs(goal_x - self.nav_x) > xy_threshold
            or abs(goal_y - self.nav_y) > xy_threshold
            or abs(goal_theta - self.heading) > math.radians(theta_threshold)
        ):
            if self.proc_count >= 2:
                self.end_process("Navigation following aborted!")
                goal_handle.abort()
                return FollowPath.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Navigation following canceled!")
                goal_handle.canceled()
                return FollowPath.Result()

            self.send_waypoint(goal_x, goal_y, goal_theta)
            feedback_msg.x = self.nav_x
            feedback_msg.y = self.nav_y
            feedback_msg.theta = self.heading
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(self.timer_period)

        self.end_process("Navigation following completed!")
        goal_handle.succeed()
        return FollowPath.Result(is_finished=True)

    #TODO: shouldn't change much, returning position of robot
    def send_waypoint(self, x, y, angular):
        control_msg = ControlMessage()
        control_msg.priority = 1
        control_msg.linear_control_mode = ControlMessage.WORLD_POSITION
        control_msg.angular_control_mode = ControlMessage.WORLD_POSITION
        control_msg.x = x
        control_msg.y = y
        control_msg.angular = angular
        self.control_pub.publish(control_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()

    #TODO: check if needed for the navigation server
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
