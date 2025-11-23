#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import time

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import CircularPID, PIDController
from all_seaing_controller.potential_field import PotentialField
from all_seaing_interfaces.action import Waypoint
from all_seaing_interfaces.msg import ControlOption
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA
from tf_transformations import quaternion_from_euler


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

        self.avoid_max_dist = (
            self.declare_parameter("avoid_max_dist", 5.0)
            .get_parameter_value()
            .double_value
        )

        self.avoid_vel_coeff = (
            self.declare_parameter("avoid_vel_coeff", 0.0)
            .get_parameter_value()
            .double_value
        )

        self.vel_marker_scale = (
            self.declare_parameter("vel_marker_scale", 1.0)
            .get_parameter_value()
            .double_value
        )
        
        self.min_goal_weight = 0.3
        self.lookahead_scale = 0.05

        # --------------- SUBSCRIBERS, PUBLISHERS, AND SERVERS ---------------#

        self.waypoint_server = ActionServer(
            self,
            Waypoint,
            "waypoint",
            execute_callback=self.waypoint_callback,
            cancel_callback=self.default_cancel_callback,
        )
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)
        
        # for obstacle avoidance
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "/point_cloud/filtered_obs", self.point_cloud_cb, qos_profile_sensor_data
        )

        self.lidar_point_cloud = None

        self.controller_marker_pub = self.create_publisher(
            MarkerArray, "controller_markers", 10
        )

        # --------------- PID CONTROLLERS ---------------#

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)

        self.lookahead_x_pid = PIDController(*Kpid_x)
        self.lookahead_y_pid = PIDController(*Kpid_y)

        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()
    
    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = msg

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
        self.lookahead_x_pid.reset()
        self.lookahead_y_pid.reset()

    def set_pid_setpoints(self, x, y, theta, lookahead_x, lookahead_y):
        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(theta)
        self.lookahead_x_pid.setpoint(lookahead_x)
        self.lookahead_y_pid.setpoint(lookahead_y)

    def update_pid(self, x, y, heading):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.x_pid.update(x, dt)
        self.y_pid.update(y, dt)
        self.lookahead_x_pid.update(x, dt)
        self.lookahead_y_pid.update(y, dt)
        self.theta_pid.update(heading, dt)
        self.prev_update_time = self.get_clock().now()

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel

        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel
    
    def vel_to_marker(self, vel, scale=1.0, rgb=(1.0, 0.0, 0.0), id=0):
        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(0, 0, math.atan2(vel[1], vel[0]))
        return Marker(
            type=Marker.ARROW,
            header=Header(frame_id=self.robot_frame_id),
            pose=Pose(orientation=orientation),
            scale=Vector3(x=scale*math.sqrt(vel[0]**2+vel[1]**2), y=0.15, z=0.15),
            color=ColorRGBA(a=1.0, r=rgb[0], g=rgb[1], b=rgb[2]),
            id=id,
        )
    
    def send_stop_cmd(self):
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = 0.0
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = 0.0
        self.control_pub.publish(control_msg)

    def control_loop(self, nav_x, nav_y, heading):
        self.update_pid(nav_x, nav_y, heading)
        dist_to_goal = (nav_x - self.goal_x) * (nav_x - self.goal_x) + (nav_y - self.goal_y) * (nav_y - self.goal_y)
        dist_to_lookahead = (nav_x - self.lookahead_x) * (nav_x - self.lookahead_x) + (nav_y - self.lookahead_y) * (nav_y - self.lookahead_y)
        goal_weight = max(self.min_goal_weight, dist_to_goal / (dist_to_lookahead * self.lookahead_scale + dist_to_goal))
        x_output = self.x_pid.get_effort() * goal_weight + self.lookahead_x_pid.get_effort() * (1 - goal_weight)
        y_output = self.y_pid.get_effort() * goal_weight + self.lookahead_y_pid.get_effort() * (1 - goal_weight)
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output * math.cos(heading) + y_output * math.sin(heading)
        y_vel = y_output * math.cos(heading) - x_output * math.sin(heading)

        marker_array = MarkerArray()
        marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 1.0, 0.0), id=0))

        # obstacle avoidance
        avoid_x_vel, avoid_y_vel = 0.0, 0.0
        if self.avoid_obs and (self.lidar_point_cloud is not None) and (self.lidar_point_cloud.width > 0):
            # TODO possibly add the setpoint as a goal in the avoiding velocity & weighted average w/ sum of weights 1, but that will mess w/ the PID more & needs more tuning
            avoid_x_vel, avoid_y_vel = PotentialField(self.lidar_point_cloud, self.avoid_max_dist).sketchy_gradient_descent_step()
            # self.get_logger().info(f'{self.lidar_point_cloud.width} points, unscaled avoiding vel: {avoid_x_vel, avoid_y_vel}')
            avoid_x_vel *= self.avoid_vel_coeff
            avoid_y_vel *= self.avoid_vel_coeff
            x_vel += avoid_x_vel
            y_vel += avoid_y_vel

        marker_array.markers.append(self.vel_to_marker((avoid_x_vel, avoid_y_vel), scale=self.vel_marker_scale, rgb=(1.0, 0.0, 0.0), id=1))
        marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 0.0, 1.0), id=2))

        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)
        self.controller_marker_pub.publish(marker_array)

    def waypoint_callback(self, goal_handle):
        self.start_process("Waypoint following started!")

        xy_threshold = goal_handle.request.xy_threshold
        theta_threshold = goal_handle.request.theta_threshold
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        goal_lookahead_x = goal_handle.request.lookahead_x
        goal_lookahead_y = goal_handle.request.lookahead_y
        is_stationary = goal_handle.request.is_stationary
        self.avoid_obs = goal_handle.request.avoid_obs

        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_lookahead_x = goal_lookahead_x
        self.goal_lookahead_y = goal_lookahead_y

        nav_x, nav_y, heading = self.get_robot_pose()
        if goal_handle.request.ignore_theta:
            goal_theta = math.atan2(goal_y - nav_y, goal_x - nav_x)
        else:
            goal_theta = goal_handle.request.theta

        self.visualize_waypoint(goal_x, goal_y)

        self.reset_pid()
        self.set_pid_setpoints(goal_x, goal_y, goal_theta, goal_lookahead_x, goal_lookahead_y)
        while (
            not self.x_pid.is_done(nav_x, xy_threshold)
            or not self.y_pid.is_done(nav_y, xy_threshold)
            or not (is_stationary or self.theta_pid.is_done(heading, math.radians(theta_threshold)))
            or is_stationary
        ):

            if self.should_abort():
                self.send_stop_cmd()
                self.end_process("Waypoint following aborted!")
                goal_handle.abort()
                return Waypoint.Result()

            if goal_handle.is_cancel_requested:
                self.send_stop_cmd()
                self.end_process("Waypoint following canceled!")
                self.send_stop_cmd()
                goal_handle.canceled()
                return Waypoint.Result()

            nav_x, nav_y, heading = self.get_robot_pose()
            self.control_loop(nav_x, nav_y, heading)
            time.sleep(TIMER_PERIOD)

        self.send_stop_cmd()
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
