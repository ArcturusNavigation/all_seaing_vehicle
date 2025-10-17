#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import time

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import CircularPID, PIDController
from all_seaing_interfaces.action import Waypoint
from all_seaing_interfaces.msg import ControlOption


from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

TIMER_PERIOD = 1 / 60

class ControllerServer(ActionServerBase):
    def __init__(self):
        super().__init__("controller_server")

        # --------------- PARAMETERS ---------------#

        Kpid_x = (
            self.declare_parameter("Kpid_x", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_y = (
            self.declare_parameter("Kpid_y", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_theta = (
            self.declare_parameter("Kpid_theta", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.max_vel = (
            self.declare_parameter("max_vel", [4.0, 2.0, 1.0])
            .get_parameter_value()
            .double_array_value
        )

        # --------------- SUBSCRIBERS, PUBLISHERS, AND SERVERS ---------------#

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            execute_callback=self.waypoint_callback,
            cancel_callback=self.default_cancel_callback,
        )
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)

        # --------------- PID CONTROLLERS ---------------#

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()


    def visualize_waypoint(self, x, y):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = self.marker_ns
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.4
        marker_msg.scale.y = 0.4
        marker_msg.scale.z = 8.0
        marker_msg.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 2.0
        self.marker_pub.publish(marker_msg)

    def reset_pid(self):
        self.prev_update_time = self.get_clock().now()
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()

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

    def control_loop(self, nav_x, nav_y, heading):
        self.update_pid(nav_x, nav_y, heading)
        x_output = self.x_pid.get_effort()
        y_output = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output * math.cos(heading) + y_output * math.sin(heading)
        y_vel = y_output * math.cos(heading) - x_output * math.sin(heading)

        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)

    def waypoint_callback(self, goal_handle):
        self.start_process("Waypoint following started!")

        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        is_stationary = goal_handle.request.is_stationary

        nav_x, nav_y, heading = self.get_robot_pose()
        if goal_handle.request.ignore_theta:
            goal_theta = math.atan2(goal_y - nav_y, goal_x - nav_x)
        else:
            goal_theta = goal_handle.request.theta

        self.visualize_waypoint(goal_x, goal_y)

        self.reset_pid()
        self.set_pid_setpoints(goal_x, goal_y, goal_theta)
        while (
            not self.x_pid.is_done(nav_x, xy_threshold)
            or not self.y_pid.is_done(nav_y, xy_threshold)
            or not (is_stationary or self.theta_pid.is_done(heading, math.radians(theta_threshold)))
            or is_stationary
        ):

            if self.should_abort():
                self.end_process("Waypoint following aborted!")
                goal_handle.abort()
                return Waypoint.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Waypoint following canceled!")
                goal_handle.canceled()
                return Waypoint.Result()

            nav_x, nav_y, heading = self.get_robot_pose()
            self.control_loop(nav_x, nav_y, heading)
            time.sleep(TIMER_PERIOD)

        self.end_process("Waypoint following completed!")
        goal_handle.succeed()
        return Waypoint.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
