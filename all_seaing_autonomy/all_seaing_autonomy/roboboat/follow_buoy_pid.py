#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import Task
from all_seaing_controller.pid_controller import PIDController, CircularPID
from ament_index_python.packages import get_package_share_directory
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption, ObstacleMap
from all_seaing_common.action_server_base import ActionServerBase
from task_server_base import TaskServerBase

from sensor_msgs.msg import CameraInfo

import os
import yaml
import time
import math

class FollowBuoyPID(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "follow_path_pid_server", action_name = "follow_buoy_pid", timer_period = 1 / 30.0)

        # Replaced by obstacle map
        self.bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray,
            "bounding_boxes",
            self.bbox_callback,
            10
        )

        # New obstacle map node
        # Will use type label and local point
        self.bbox_sub_new = self.create_subscription(
            ObstacleMap,
            "obstacle_map/labeled",
            self.bbox_callback_new,
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

        self.green_labels = set()
        self.red_labels = set()
        self.yellow_labels = set()

        self.result = False
        self.seen_first_buoy = False

        self.scale_right = 1.0


        self.declare_parameter(
            "color_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )
        self.declare_parameter("right_color", "green")
        # Integers, feet apart
        self.declare_parameter("max_distance_apart", 3)
        self.declare_parameter("min_distance_apart", 2)
        self.declare_parameter("meters_feet_conversion", 0.3048)
        self.declare_parameter("front_limit", 0.0)
        self.waypoint_x = None
        self.waypoint_y = None
        self.red_green_ratio = None

        self.right_color = self.get_parameter("right_color").get_parameter_value().string_value
        self.max_distance_apart = self.get_parameter("max_distance_apart").get_parameter_value().integer_value
        self.min_distance_apart = self.get_parameter("min_distance_apart").get_parameter_value().integer_value
        self.meters_feet_conversion = self.get_parameter("meters_feet_conversion").get_parameter_value().double_value
        self.front_limit = self.get_parameter("front_limit").get_parameter_value().double_value

        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)
        # hardcoded from reading YAML
        
        if self.is_sim:
            self.green_labels.add(label_mappings["green"])
            self.red_labels.add(label_mappings["red"])
            self.yellow_labels.add(label_mappings["black"])
        else:
            self.green_labels.add(label_mappings["green_buoy"])
            self.red_labels.add(label_mappings["red_buoy"])
            self.yellow_labels.add(label_mappings["yellow_buoy"])



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

    def control_loop(self):
        # if self.width is None or len(self.obstacleboxes) == 0:
        #     self.get_logger().info(f"no obstalces or zero width {self.width}, {len(self.obstacleboxes)}")
        #     return

        # Access point through name.point.x, etc.?
        red_location = None
        red_area = 0

        green_location = None
        green_area = 0

        # yellow_center_x = None
        yellow_area = 0
        # yellow_left = None
        # yellow_right = None
        yellow_location = None

        # handle balancing box sizes later ?
        # ex. same size but offset should be y shift
        # but diff size should be a rotation and probably
        # a bit of a y shift too?

        # Still use this logic to find the largest buoy, since probably most relative to path
        forward = False
        for box in self.obstacleboxes:
            area = (box.bbox_max.x - box.bbox_min.x) * (box.bbox_max.y - box.bbox_min.y)
            location = box.local_point
            if box.label in self.green_labels and area > green_area:
                self.get_logger().info('GREEN THERE')
                green_area = area
                green_location = location
            elif box.label in self.red_labels and area > red_area:
                self.get_logger().info('RED THERE')
                red_area = area
                red_location = location
            elif box.label in self.yellow_labels and area > yellow_area:
                yellow_area = area
                yellow_location = location

        # if we only see one of red / green, rotate
        # if we see neither, log + kill
        # if red_center_x is None or green_center_x is None:
        #     if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 1.0:
        #         self.get_logger().info("no more buoys killing")
        #         self.result = True
        #     return


        # if red_center_x is not None and green_center_x is not None:
        #     self.red_green_ratio = red_area / green_area
        # keep going in the same direction if the ratio is too far off.
        # if self.red_green_ratio is not None and (self.red_green_ratio > 0.5 or self.red_green_ratio < 0.5):
        #     yaw0 = True
        #     return

        # Need to change this logic to something less crude, pure pursuit was suggested in task description
        # Currently uses location in image, need to change to each obstacle's local points

        # can update, not impt rn
        # left_x = red_center_x
        # right_x = green_center_x
        # img_ctr = self.width / 2.0

        # Used when only see one buoy
        correction_value = (self.max_distance_apart + self.min_distance_apart)/2 * self.meters_feet_conversion
        self.waypoint_x = None
        self.waypoint_y = None
        red_x = None
        red_y = None
        green_x = None
        green_y = None

        if green_location is not None or red_location is not None:
            # Set imaginary location for any buoys we do not see
            if green_location is None:
                self.get_logger().info("green was None")
                red_y = red_location.point.y
                red_x = red_location.point.x
                if self.right_color == "green":
                    green_y = red_y - correction_value
                    green_x = red_x
                elif self.right_color == "red":
                    green_y = red_y + correction_value
                    green_x = red_x
            elif red_location is None:
                self.get_logger().info("red was None")
                green_y = green_location.point.y
                green_x = green_location.point.x
                if self.right_color == "green":
                    red_y = green_y + correction_value
                    red_x = green_x
                if self.right_color == "red":
                    red_y = green_y - correction_value
                    red_x = green_x
            else:
                green_y = green_location.point.y
                green_x = green_location.point.x
                red_y = red_location.point.y
                red_x = red_location.point.x

            # Main logic of the follow the path
            # If no yellows, just the midpt
            if yellow_location == None:
                self.waypoint_y = (red_y + green_y)/2
                self.waypoint_x = (red_x + green_x)/2
            # If yellow, check if relevant and have two different cases if is
            else:
                yellow_x = yellow_location.point.x
                yellow_y = yellow_location.point.y
                front = None
                # Sets whether yellow is in front or behind of the line of red, green intersection
                if self.right_color == "green":
                    front = self.ccw((green_y, green_x), (yellow_y, yellow_x), (red_y, red_x))
                else:
                    front = self.ccw((red_y, red_x), (yellow_y, yellow_x), (green_y, green_x))
                # Checks if yellow is between the green and red buoys
                if (self.right_color == "green" and red_y < yellow_y < green_y) or (self.right_color == "red" and green_y < yellow_y < red_y):
                    # If in front, finds intersection of red, green buoy line and its perpendicular line passing through yellow's location
                    if front:
                        red_to_yellow = (yellow_x - red_x, yellow_y -red_y)
                        red_to_green = (green_x - red_x, green_y - red_y)
                        dot_prod = red_to_yellow[0] * red_to_green[0] + red_to_yellow[1] * red_to_green[1]
                        const_fact = dot_prod / self.dist_squared(red_to_green)
                        intersection_x = red_to_green[0] * const_fact + red_x
                        intersection_y = red_to_green[1] * const_fact + red_y


                        square_distance_red = (intersection_y - red_y)**2 + (intersection_x - red_x)**2
                        square_distance_green = (intersection_y - green_y)**2 + (intersection_x - green_x)**2
                        if square_distance_red >= square_distance_green:
                            self.waypoint_x = (red_x + intersection_x)/2
                            self.waypoint_y = (red_y + intersection_y)/2
                        else:
                            self.waypoint_x = (green_x + intersection_x)/2
                            self.waypoint_y = (green_y + intersection_y)/2
                    # If behind, then just finds largest red, yellow or green, yellow dist and takes the midpt
                    else:
                        red_to_yellow = (yellow_x - red_x, yellow_y -red_y)
                        green_to_yellow = (yellow_x - green_x, yellow_y -green_y)
                        ry_dist = self.dist_squared(red_to_yellow)
                        gy_dist = self.dist_squared(green_to_yellow)
                        if ry_dist >= gy_dist:
                            self.waypoint_x = (red_x + yellow_x)/2
                            self.waypoint_y = (red_y + yellow_y)/2
                        else:
                            self.waypoint_x = (green_x + yellow_x)/2
                            self.waypoint_y = (green_y + yellow_y)/2
                # If not in between the red and green buoys, treats it like there is no obstacle
                else:
                    self.waypoint_y = (red_y + green_y)/2
                    self.waypoint_x = (red_x + green_x)/2


        yaw0 = False
        # if self.waypoint_x is None or (math.sqrt(self.waypoint_x**2)+ math.sqrt(self.waypoint_y**2)) < self.front_limit:
        if green_x is None and red_x is None:
            yaw0 = True
            if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 100.0:
                self.get_logger().info("no more buoys killing")
                # wait 1 second, then send a stopping control msg (in case we haven't fully passed the buoys)
                time.sleep(1)

                self.mark_successful()
            self.get_logger().info('KILLING THRUSTERS')
            control_msg = ControlOption()
            control_msg.priority = 1
            self.get_logger().info(f'SENDING COMMAND: {control_msg}')
            self.control_pub.publish(control_msg)
            return
        else:
            self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        # POINTS TO GIVE PID: self.waypoint_y and self.waypoint_x
        self.get_logger().info(f"green x: {green_x}, green y: {green_y}")
        self.get_logger().info(f"red x: {red_x}, red y: {red_y}")
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



def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPID()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
