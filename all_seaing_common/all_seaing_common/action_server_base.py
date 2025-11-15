from abc import ABC
import rclpy
from rclpy.action import CancelResponse
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from math import cos, sin
from tf_transformations import euler_from_quaternion
from threading import Semaphore, Event
from visualization_msgs.msg import Marker


class ActionServerBase(ABC, Node):
    """
    Abstract base class for action servers. Includes useful robot pose, marker deletion,
    and multithreaded process management functions.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.marker_ns = node_name

        # --------------- PARAMETERS ---------------#

        self.global_frame_id = (
            self.declare_parameter("global_frame_id", "map")
            .get_parameter_value()
            .string_value
        )
        self.robot_frame_id = (
            self.declare_parameter("robot_frame_id", "base_link")
            .get_parameter_value()
            .string_value
        )

        # --------------- SUBSCRIBERS AND PUBLISHERS ---------------#

        self.marker_pub = self.create_publisher(Marker, "action_marker", 10)

        # --------------- TF SETUP ---------------#

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------- PROCESS MANAGEMENT ---------------#

        self.proc_semaphore = Semaphore(1)
        self.proc_cancel_evt = Event()

    def default_cancel_callback(self, cancel_request):
        """Canceling has no extra functionalities by default"""
        return CancelResponse.ACCEPT

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

        
    @property
    def robot_pos(self):
        '''
        Gets the robot position as a tuple (x,y)
        '''
        position = self.get_robot_pose()[0:2]
        return (float(position[0]), float(position[1]))

    @property
    def robot_dir(self):
        '''
        Gets the robot direction as a tuple, containing the unit vector in the same direction as heading
        '''
        heading = self.get_robot_pose()[2]
        return (cos(heading), sin(heading))

    def get_robot_pose(self):
        """
        Return robot pose as a tuple of (x, y, heading).
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame_id,
                self.robot_frame_id,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.global_frame_id} to {self.robot_frame_id}: {ex}')
            return 0, 0, 0
        
        x = t.transform.translation.x
        y = t.transform.translation.y
        _, _, heading = euler_from_quaternion(
            [
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]
        )
        return x, y, heading
