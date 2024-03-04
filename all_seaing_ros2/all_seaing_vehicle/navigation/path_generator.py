#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseArray
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path


class PathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")

        # Declare parameters
        self.declare_parameter("update_frequency", 2.0)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("sample_rate", 0.05)
        update_frequency = self.get_parameter("update_frequency").value
        self.publish_path = self.get_parameter("publish_path").value
        self.sample_rate = self.get_parameter("sample_rate").value

        # Create action client, timer, subscriber, and publisher
        self.action_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.create_timer(update_frequency, self.send_goal)
        if self.publish_path:
            self.path_pub = self.create_publisher(Path, "nav2_path", 10)
        self.waypoint_pub = self.create_publisher(PoseArray, "/waypoints", 10)

    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = "odom"
        goal_msg.goal.pose.position.x = 8.0
        goal_msg.goal.pose.position.y = 40.0
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        # Goal accepted => go to get_result_callback
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        path = future.result().result.path

        # Publish path if publish_path parameter is True
        if self.publish_path: self.path_pub.publish(path)

        # Sample path by sample_rate and send to waypoint sender
        self.get_logger().info(f"{len(path.poses)}")
        sample_interval = max(1, round(len(path.poses) * self.sample_rate))
        wpt_msg = PoseArray()
        wpt_msg.header = path.header
        for pose_stamped in path.poses[::sample_interval]:
            pose = pose_stamped.pose
            wpt_msg.poses.append(pose)
        self.waypoint_pub.publish(wpt_msg)


def main(args=None):
    rclpy.init(args=args)
    action_client = PathGenerator()
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
