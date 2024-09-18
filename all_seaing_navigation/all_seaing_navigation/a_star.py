# A* star template from RSS (WIP) :)

import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, PointStamped
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory, Map
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
import time
import numpy as np

import numpy as np
# import cProfile

class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")
        self.declare_parameter('clicked_point_topic',"default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value

        # 2-D grid map, each cell represents the probability of occupancy
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.get_logger().info("initialized")

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.point_sub = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.update_path,
            10
        )

        # self.tree_pub = self.create_publisher(MarkerArray, '/tree', 10)           ## REMOVE THIS

        #Line Trajectory class in utils.py
        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.occ_map = None
        self.points = []
        self.s = None
        self.t = None


    def map_cb(self, msg):

        self.get_logger().info("starting")
        self.occ_map = Map(msg)


    def update_path(self,msg):
        pass

    # Converts quaternion orientation into Euler angles
    def pose_cb(self, pose):
        """
        New initial pose (PoseWithCovarianceStamped)
        """
        self.get_logger().info("Intial pose set, everything else reset")
        orientation = euler_from_quaternion((
        pose.pose.pose.orientation.x,
        pose.pose.pose.orientation.y,
        pose.pose.pose.orientation.z,
        pose.pose.pose.orientation.w))
        self.s = [pose.pose.pose.position.x,pose.pose.pose.position.y,orientation[2]]


    def goal_cb(self, msg):
        """
        New goal pose (PoseStamped)
        """
        if True or (self.s is not None and len(self.points) == 3):
            self.get_logger().info("Goal")
            orientation = euler_from_quaternion((
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w))
            self.t = [msg.pose.position.x,msg.pose.position.y,orientation[2]]

            # Plan the whole path
            self.full_path()

    def full_path(self):
        total_path = []
        if self.s is not None:

            last_p = self.s
            for p in self.points:
                path = self.plan_path(last_p, p)

                if path is not None:
                    total_path.extend(path)
                    last_p = p
                else:
                    return

            if self.t is not None:
                path = self.plan_path(last_p, self.t)

                if path is not None:
                    total_path.extend(path)
                else:
                    return

            self.publish_path(total_path)
        else:
            self.get_logger().info("Choose initial state first")


    def plan_path(self, start_point, end_point):
        """
        start_point s: Ros2 Point
        end_point t: Ros2 Point
        """
        pass

    def publish_path(self,path):

        self.trajectory.updatePoints(path)

        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

    def publish_marker_array(self, publisher, xa, ya, rgb=[1.0,0.0,0.0]):
        """
        converts the point array from car to global frame and publishes the markerarray
        """
        markers = []
        count = 0
        for x, y in zip(xa, ya):
            marker = self.to_marker((x, y), count, rgb=rgb)
            markers.append(marker)
            count+=1

        markerarray = MarkerArray()
        markerarray.markers = markers
        publisher.publish(markerarray)

    def to_marker(self,position,id = 1,rgb=[1.0,0.0,0.0],scale=0.25):
        marker = Marker()
        marker.header.frame_id = "/map"  # Set the frame id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "/followed_trajectory/trajectory"
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]

        return marker


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
