from abc import ABC
from rclpy.action import CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from threading import Semaphore, Event
from visualization_msgs.msg import Marker


class ActionServerBase(ABC, Node):
    """
    Abstract base class for action servers. Includes useful odometry, marker deletion,
    and multithreaded process management functions.
    """

    def __init__(self, node_name, marker_ns, timer_period):
        super().__init__(node_name)
        self.marker_ns = marker_ns
        self.timer_period = timer_period

        # --------------- PARAMETERS ---------------#

        self.global_frame_id = (
            self.declare_parameter("global_frame_id", "odom")
            .get_parameter_value()
            .string_value
        )

        # --------------- SUBSCRIBERS AND PUBLISHERS ---------------#

        self.group = MutuallyExclusiveCallbackGroup()
        self.odom_sub = self.create_subscription(
            Odometry,
            "odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.group,
        )
        self.marker_pub = self.create_publisher(Marker, "action_marker", 10)

        # --------------- MEMBER VARIABLES ---------------#

        self.nav_x = 0.0
        self.nav_y = 0.0
        self.heading = 0.0

        # --------------- PROCESS MANAGEMENT ---------------#

        self.proc_semaphore = Semaphore(1)
        self.proc_cancel_evt = Event()

    def default_cancel_callback(self, cancel_request):
        """Canceling has no extra functionalities by default"""
        return CancelResponse.ACCEPT

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
        """
        Manages the beginning of new client request.
        Prevents multiple processes from running at the same time using a semaphore.
        Always call this function at the beginning of an action server execute callback.
        """
        self.proc_cancel_evt.set()
        self.proc_semaphore.acquire()
        self.proc_cancel_evt.clear()
        if msg:
            self.get_logger().info(msg)

    def end_process(self, msg=None):
        """
        Manages the ending of a running client request.
        Always call this function when an action server execute callback gets canceled,
        aborted, or successfully completed.
        """
        self.delete_all_marker()
        if msg:
            self.get_logger().info(msg)
        self.proc_semaphore.release()

    def should_abort(self):
        """
        Checks if the process should stop due to a new request.
        """
        return self.proc_cancel_evt.is_set()

    def delete_all_marker(self):
        """
        Delete all visualization markers in the same namespace.
        """
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = self.marker_ns
        marker_msg.action = Marker.DELETEALL
        self.marker_pub.publish(marker_msg)
