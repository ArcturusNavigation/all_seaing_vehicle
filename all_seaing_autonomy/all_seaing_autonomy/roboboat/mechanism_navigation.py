#!/usr/bin/env python3

# NOTE: Copied from follow_buoy_pid.py so need to make a lot of changes

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import Task
from all_seaing_controller.pid_controller import PIDController, CircularPID
from ament_index_python.packages import get_package_share_directory
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption, ObstacleMap, LabeledObjectPlaneArray
from all_seaing_common.action_server_base import ActionServerBase
from sensor_msgs.msg import CameraInfo

import os
import yaml
import time
import math

class MechanismNavigation(ActionServerBase):
    def __init__(self):
        super().__init__("mechanism_navigation")

        self._action_server = ActionServer(
            self,
            Task,
            "mechanism_navigation",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        # Replaced by obstacle map
        self.bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray,
            "bounding_boxes",
            self.bbox_callback,
            10
        )

        # New obstacle map node
        # Will use type label and global point
        self.bbox_sub_new = self.create_subscription(
            ObstacleMap,
            "obstacle_map/labeled",
            self.bbox_callback_new,
            10
        )

        # New, new subscription??
        self.plane_box = self.create_subscription(
            LabeledObjectPlaneArray,
            "object_planes",
            self.plane_box,
            10
        )

        self.intrinsics_sub = self.create_subscription(
            CameraInfo,
            "camera_info",
            self.intrinsics_callback,
            10
        )

        self.control_pub = self.create_publisher(
            ControlOption,
            "control_options",
            10
        )

        Kpid_x = (
            self.declare_parameter("Kpid_x", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_y = (
            self.declare_parameter("Kpid_y", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_theta = (
            self.declare_parameter("Kpid_theta", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.max_vel = (
            self.declare_parameter("max_vel", [2.0, 2.0, 0.4])
            .get_parameter_value()
            .double_array_value
        )

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()


        # pid_vals = (
        #     self.declare_parameter("pid_vals", [0.003, 0.0, 0.0])
        #     .get_parameter_value()
        #     .double_array_value
        # )


        self.declare_parameter("forward_speed", 5.0)
        self.declare_parameter("max_yaw", 1.0)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        # self.pid = PIDController(*pid_vals)
        # self.pid.set_effort_min(-self.max_yaw_rate)
        # self.pid.set_effort_max(self.max_yaw_rate)
        # self.prev_update_time = self.get_clock().now()
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9


        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)

        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        # update from subs
        self.height = None
        self.width = None
        self.bboxes = [] # To be commented out
        self.obstacleboxes = []
        self.planeboxes = []

        self.timer_period = 1 / 30.0

        # Parameter that set what boat we got to first
        self.declare_parameter("cross_first", True)
        self.cross_first = self.get_parameter("cross_first").get_parameter_value().bool_value
        # Parameters for when is acceptable to have "reached" boat
        self.declare_parameter("x_threshold", 5)
        self.declare_parameter("y_threshold", 5)
        self.x_threshold = self.get_parameter("x_threshold").get_parameter_value().double_value
        self.y_threshold = self.get_parameter("y_threshold").get_parameter_value().double_value

        self.declare_parameter(
            "color_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )
        self.waypoint_x = None
        self.waypoint_y = None

        # Only want this to happen on init, check that this is the case
        self.triangle_storage_x = None
        self.triangle_storage_y = None
        self.cross_storage_x = None
        self.cross_storage_y = None

        # Declares if have completed first step, should only happen on init
        self.cross_done = False

        # Adds objects with cross or triangle in name to set but is not correct
        # Ask/Look into how this should be done
        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)
        # hardcoded from reading YAML

        # Don't know if strings are correct here
        if self.is_sim:
            self.triangle_labels.add(label_mappings["triangle"])
            self.cross_labels.add(label_mappings["cross"])
        else:
            self.triangle_labels.add(label_mappings["triangle"])
            self.cross_labels.add(label_mappings["cross"])


    def set_pid_setpoints(self, x, y, theta):
        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(theta)

    def update_pid(self, x, y, heading):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.x_pid.update(x, dt)
        self.y_pid.update(y, dt)
        self.theta_pid.update(heading, dt)
        self.prev_update_time = self.get_clock().now()

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel

        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel

    def intrinsics_callback(self, msg):
        self.height = msg.height
        self.width = msg.width

    def dist_squared(self, vect):
        return vect[0] * vect[0] + vect[1] * vect[1]

    def ccw(self, right, yellow, left):
        """Return True if the points a, b, c are counterclockwise, respectively"""
        area = (
            right[0] * yellow[1]
            + yellow[0] * left[1]
            + left[0] * right[1]
            - right[1] * yellow[0]
            - yellow[1] * left[0]
            - left[1] * right[0]
        )
        return area > 0

    # Replaced by obstacle map
    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    # New version with obstacles
    def bbox_callback_new(self, msg):
        self.obstacleboxes = msg.obstacles

    # New new version with plane objects?
    def plane_box_callback(self, msg):
        self.planeboxes = msg.object_planes

    def control_loop(self):
        # if self.width is None or len(self.obstacleboxes) == 0:
        #     self.get_logger().info(f"no obstalces or zero width {self.width}, {len(self.obstacleboxes)}")
        #     return

        # Access point through name.point.x, etc.?
        triangle_location = None
        triangle_area = 0

        cross_location = None
        cross_area = 0

        # handle balancing box sizes later ?
        # ex. same size but offset should be y shift
        # but diff size should be a rotation and probably
        # a bit of a y shift too?

        # Logic to find the largest cross and triangle just in case get noise?
        # Choose the last triangle/cross in the list
        # Could use size (type vector 3) to choose closest in theory
        forward = False
        for box in self.planeboxes:
            # area = (box.bbox_max.x - box.bbox_min.x) * (box.bbox_max.y - box.bbox_min.y)
            location = box.normal_ctr # Type Pose (point, Quaternian)
            if box.label in self.triangle_labels: # and area > triangle_area:
                self.get_logger().info('TRIANGLE THERE')
                # triangle_area = area
                triangle_location = location.point
            elif box.label in self.cross_labels: # and area > cross_area:
                self.get_logger().info('CROSS THERE')
                # cross_area = area
                cross_location = location.point

        # Initializes values
        self.waypoint_x = None
        self.waypoint_y = None
        self.waypoint_cross_x = None
        self.waypoint_cross_y = None
        self.waypoint_triangle_x = None
        self.waypoint_triangle_y = None
        cross_x = None
        cross_y = None
        triangle_x = None
        triangle_y = None

        # Handles cases where only have one and maybe build into logic of storing location somehow
        if cross_location is not None or triangle_location is not None:
            # Set imaginary location for any buoys we do not see
            if cross_location is None:
                self.get_logger().info("cross was None")
                triangle_y = triangle_location.point.y
                triangle_x = triangle_location.point.x
                cross_y = self.cross_storage_y
                cross_x = self.cross_storage_x
            elif triangle_location is None:
                self.get_logger().info("triangle was None")
                cross_y = cross_location.point.y
                cross_x = cross_location.point.x
                triangle_y = self.triangle_storage_y
                triangle_x = self.triangle_storage_x
            else:
                triangle_y = triangle_location.point.y
                triangle_x = triangle_location.point.x
                cross_y = cross_location.point.y
                cross_x = cross_location.point.x

        # Reset global variables for storage
        self.triangle_storage_x = triangle_x
        self.triangle_storage_y = triangle_y
        self.cross_storage_x = cross_x
        self.cross_storage_y = cross_y

        # Main logic of the mechanism navigation
        # From here assumes no positions are NONE
        # Needs to know when to stop and then switch to other
        # Captures robot position only
        # Doesn't do heading so might need to do something with that later
        position = self.get_robot_pose()[0:1]
        robot_x = position[1]
        robot_y = position[0]
        if abs(triangle_x - robot_x) <= self.x_threshold:
            self.waypoint_triangle_x = robot_x

        if abs(cross_x - robot_x) <= self.x_threshold:
            self.waypoint_cross_x = robot_x

        if abs(triangle_y - robot_y) <= self.y_threshold:
            self.waypoint_triangle_y = robot_y

        if abs(cross_y - robot_y) <= self.y_threshold:
            self.waypoint_cross_y = robot_y

        if self.cross_first and not self.cross_done:
            if self.waypoint_cross_x == robot_x and self.waypoint_cross_y == robot_y:
                self.cross_done = True
                # Add in wait here?
            else:
                self.waypoint_y = self.waypoint_cross_y
                self.waypoint_x = self.waypoint_cross_x
        elif self.cross_done:
            if self.waypoint_triangle_x == robot_x and self.waypoint_triangle_y == robot_y:
                # Place holder so no error REMOVE
                self.waypoint_triangle_x = 0
                # Can terminate
                # Can add in wait
            else:
                self.waypoint_x = self.waypoint_triangle_x
                self.waypoint_y = self.waypoint_triangle_y

        yaw0 = False
        # if self.waypoint_x is None or (math.sqrt(self.waypoint_x**2)+ math.sqrt(self.waypoint_y**2)) < self.front_limit:
        # In theory shouldn't happen, but maybe uncomment and have just in case
        # if cross_x is None and triangle_x is None:
        #     yaw0 = True
        #     if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 100.0:
        #         self.get_logger().info("not detecting cross or triangle")
        #         # wait 1 second, then send a stopping control msg (in case we haven't fully passed the buoys)
        #         time.sleep(1)

        #         self.result = True
        #     self.get_logger().info('KILLING THRUSTERS')
        #     control_msg = ControlOption()
        #     control_msg.priority = 1
        #     self.get_logger().info(f'SENDING COMMAND: {control_msg}')
        #     self.control_pub.publish(control_msg)
        #     return
        # else:
        #     self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        # POINTS TO GIVE PID: self.waypoint_y and self.waypoint_x
        self.get_logger().info(f"triangle x: {triangle_x}, triangle y: {triangle_y}")
        self.get_logger().info(f"cross x: {cross_x}, cross y: {cross_y}")
        self.get_logger().info(f"waypoint x: {self.waypoint_x}, waypoint y: {self.waypoint_y}")

        self.update_pid(-self.waypoint_x, -self.waypoint_y, math.atan2(-self.waypoint_y, -self.waypoint_x))
        x_output = self.x_pid.get_effort()
        y_output = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output
        y_vel = y_output

        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)


    def execute_callback(self, goal_handle):
        self.start_process("mechanism navigation pid starting")
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9
        self.set_pid_setpoints(0, 0, 0) #want angle to point to be 0 radians

        while not self.result:

            if self.should_abort():
                self.end_process("aborting mechanism navigation")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("cancelling mechanism navigation")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)

        self.end_process("mechanism navigation pid completed!")
        goal_handle.succeed()
        return Task.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = MechanismNavigation()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
