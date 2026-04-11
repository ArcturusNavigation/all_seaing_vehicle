#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import CircularPID
from all_seaing_interfaces.action import FollowPath
from all_seaing_interfaces.msg import ControlOption, ContinuousWaypoint, WaypointStatus
from all_seaing_navigation.planner_executor import PlannerExecutor
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from all_seaing_controller.potential_field import PotentialField

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header

from threading import Semaphore, Event
import math
import time

TIMER_PERIOD = 1 / 60


class NavigationContinuousServer(ActionServerBase):
    def __init__(self):
        super().__init__("navigation_server_continuous")

        # FollowPath action server for backward compat
        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self.follow_path_callback,
            cancel_callback=self.default_cancel_callback,
        )

        # Continuous waypoint topic interface
        self.waypoint_sub = self.create_subscription(
            ContinuousWaypoint, "continuous_waypoint", self.waypoint_callback, 10
        )
        self.status_pub = self.create_publisher(WaypointStatus, "waypoint_status", 10)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/dynamic_map", self.map_callback, 10
        )

        # Parameters
        self.default_forward_speed = (
            self.declare_parameter("default_forward_speed", 2.0)
            .get_parameter_value()
            .double_value
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
        self.rot_avoid_vel_coeff = (
            self.declare_parameter("rot_avoid_vel_coeff", 0.0)
            .get_parameter_value()
            .double_value
        )
        self.avoid_rot_vel_mag = (
            self.declare_parameter("avoid_rot_vel_mag", True)
            .get_parameter_value()
            .bool_value
        )
        self.vel_marker_scale = (
            self.declare_parameter("vel_marker_scale", 1.0)
            .get_parameter_value()
            .double_value
        )
        self.forward_dist = (
            self.declare_parameter("forward_dist", 1.0)
            .get_parameter_value()
            .double_value
        )

        # Publishers
        self.control_pub = self.create_publisher(ControlOption, "control_options", 10)
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "/point_cloud/filtered_obs", self.point_cloud_cb, qos_profile_sensor_data
        )
        self.controller_marker_pub = self.create_publisher(
            MarkerArray, "controller_markers", 10
        )

        # PID (heading only)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        # State
        self.lidar_point_cloud = None
        self.map = None
        self.planner = PlannerExecutor(self)
        self.stop_plan_semaphore = Semaphore(1)
        self.stop_plan_evt = Event()

        # Continuous waypoint state
        self.current_waypoint = None  # (x, y, forward_speed, avoid_obs)
        self.approach_origin = None   # robot position when waypoint was set
        self.approach_dir = None
        self.waypoint_reached = False
        self.xy_threshold = 0.0

        # Control loop timer (only runs when a continuous waypoint is active)
        self.control_timer = self.create_timer(TIMER_PERIOD, self.continuous_control_tick)

    # --------------- Callbacks ---------------#

    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def waypoint_callback(self, msg: ContinuousWaypoint):
        if self.current_waypoint is None:
            return
        # self.get_logger().info(f'updating continuous waypoint')
        nav_x, nav_y, _ = self.get_robot_pose()
        self.approach_origin = np.array([nav_x, nav_y])
        self.approach_dir = np.array([msg.x, msg.y]) - self.approach_origin
        # self.approach_dir /= np.linalg.norm(self.approach_dir)
        # self.current_waypoint = (msg.x, msg.y, msg.forward_speed, msg.avoid_obs)
        self.current_waypoint = (msg.x, msg.y, self.current_waypoint[2], self.current_waypoint[3])
        # self.waypoint_reached = False
        # self.reset_pid()

    # --------------- PID helpers ---------------#

    def reset_pid(self):
        self.prev_update_time = self.get_clock().now()
        self.theta_pid.reset()

    def update_pid(self, heading):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.theta_pid.update(heading, dt)
        self.prev_update_time = self.get_clock().now()

    # --------------- Velocity helpers ---------------#

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel
        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel

    def vel_to_marker(self, vel, scale=1.0, rgb=(1.0, 0.0, 0.0), id=0):
        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = quaternion_from_euler(
            0, 0, math.atan2(vel[1], vel[0])
        )
        return Marker(
            type=Marker.ARROW,
            header=Header(frame_id=self.robot_frame_id),
            pose=Pose(orientation=orientation),
            scale=Vector3(x=scale * math.sqrt(vel[0]**2 + vel[1]**2), y=0.15, z=0.15),
            color=ColorRGBA(a=1.0, r=rgb[0], g=rgb[1], b=rgb[2]),
            id=id,
        )

    def send_stop_cmd(self):
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = 0.0
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = 0.0
        self.control_pub.publish(control_msg)

    def global_to_robot(self, dx, dy, heading):
        """Transform a global-frame vector to robot frame."""
        return (
            dx * math.cos(heading) + dy * math.sin(heading),
            -dx * math.sin(heading) + dy * math.cos(heading),
        )

    # --------------- Waypoint crossing check ---------------#

    def check_waypoint_crossed(self, nav_x, nav_y, goal_x, goal_y):
        """Check if robot has crossed the perpendicular line through the goal."""
        if self.approach_origin is None:
            return False
        if np.linalg.norm(self.approach_dir) < 1e-6:
            # Waypoint is at the approach origin — use distance check
            return np.linalg.norm(np.array([nav_x - goal_x, nav_y - goal_y])) < 0.5
        return np.array([nav_x - goal_x, nav_y - goal_y]) @ self.approach_dir >= 0

    # --------------- Visualization ---------------#

    def visualize_waypoint(self, x, y):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "continuous_waypoint"
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.4
        marker_msg.scale.y = 0.4
        marker_msg.scale.z = 8.0
        marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 2.0
        marker_msg.id = 0
        self.marker_pub.publish(marker_msg)

    # --------------- Continuous control loop ---------------#

    def continuous_control_tick(self):
        """Timer callback: runs at 60 Hz when a continuous waypoint is active."""
        if self.current_waypoint is None:
            return

        goal_x, goal_y, forward_speed, avoid_obs = self.current_waypoint
        nav_x, nav_y, nav_theta = self.get_robot_pose()
        goal_vec = np.array([goal_x, goal_y])
        goal_diff = goal_vec-self.robot_pos
        dist = np.linalg.norm(goal_diff)

        # Check perpendicular crossing
        crossed = self.check_waypoint_crossed(nav_x, nav_y, goal_x, goal_y)
        if crossed:
            self.waypoint_reached = True

        # Publish status
        status_msg = WaypointStatus()
        status_msg.waypoint_reached = self.waypoint_reached
        status_msg.distance_to_waypoint = dist
        self.status_pub.publish(status_msg)

        if dist < self.xy_threshold and np.linalg.norm(self.approach_dir) >= 1e-6:
            # make it go forward until it crosses the waypoint line
            forward_goal_vec = self.forward_dist*self.approach_dir/np.linalg.norm(self.approach_dir)
            goal_vec += forward_goal_vec
            goal_diff += forward_goal_vec

        # Compute desired heading toward goal
        desired_heading = math.atan2(goal_diff[1], goal_diff[0])

        # Heading PID
        self.theta_pid.set_setpoint(desired_heading)
        self.update_pid(nav_theta)
        theta_output = self.theta_pid.get_effort()

        # Constant forward velocity in robot frame
        # x_vel = forward_speed
        x_vel = max(0.0, self.robot_dir @ goal_diff/np.linalg.norm(goal_diff))*forward_speed # so that we go faster when we are more aligned with the goal
        y_vel = 0.0

        self.visualize_waypoint(goal_vec[0], goal_vec[1])

        marker_array = MarkerArray()
        marker_array.markers.append(
            self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 1.0, 0.0), id=0)
        )

        # LiDAR obstacle avoidance
        avoid_x_vel, avoid_y_vel = 0.0, 0.0
        if avoid_obs and self.lidar_point_cloud is not None and self.lidar_point_cloud.width > 0:
            pf = PotentialField(self.lidar_point_cloud, self.avoid_max_dist)
            avoid_x_vel, avoid_y_vel = pf.sketchy_gradient_descent_step()
            avoid_x_vel *= self.avoid_vel_coeff
            avoid_y_vel *= self.avoid_vel_coeff
            # Rotational avoidance
            rot_avoid_x_vel, rot_avoid_y_vel = pf.rotational_force(vel_dir=(x_vel, y_vel))
            rot_avoid_x_vel *= self.rot_avoid_vel_coeff
            rot_avoid_y_vel *= self.rot_avoid_vel_coeff
            if self.avoid_rot_vel_mag:
                vel_mag = math.sqrt(x_vel**2 + y_vel**2)
                rot_avoid_x_vel *= vel_mag
                rot_avoid_y_vel *= vel_mag
            avoid_x_vel += rot_avoid_x_vel
            avoid_y_vel += rot_avoid_y_vel
            x_vel += avoid_x_vel
            y_vel += avoid_y_vel

        marker_array.markers.append(
            self.vel_to_marker((avoid_x_vel, avoid_y_vel), scale=self.vel_marker_scale, rgb=(1.0, 0.0, 0.0), id=1)
        )
        marker_array.markers.append(
            self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 0.0, 1.0), id=2)
        )

        # Scale and publish
        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)
        self.controller_marker_pub.publish(marker_array)

    # --------------- FollowPath backward compat ---------------#

    def start_plan(self):
        self.stop_plan_evt.set()
        self.stop_plan_semaphore.acquire()
        self.stop_plan_evt.clear()

    def should_abort_plan(self):
        return self.stop_plan_evt.is_set()

    def stopped_plan(self):
        self.stop_plan_semaphore.release()

    def generate_path(self, goal_handle, nav_x, nav_y):
        self.start_plan()
        start = Point(x=nav_x, y=nav_y)
        goal = Point(x=goal_handle.request.x, y=goal_handle.request.y)
        obstacle_tol = goal_handle.request.obstacle_tol
        goal_tol = goal_handle.request.goal_tol

        path = self.planner.plan(start, goal, obstacle_tol, goal_tol, self.should_abort_plan)
        if len(path.poses) <= 1:
            from geometry_msgs.msg import Pose as GPose
            path.poses = [GPose(position=start), GPose(position=goal)]

        self.stopped_plan()
        return path

    def follow_path_callback(self, goal_handle):
        """Backward-compatible FollowPath: uses continuous waypoint steering along A* path."""
        self.get_logger().info("FollowPath started (continuous server, backward compat)")

        if self.map is None:
            self.get_logger().info("No map available. Aborting.")
            goal_handle.abort()
            return FollowPath.Result()

        nav_x, nav_y, _ = self.get_robot_pose()
        path = self.generate_path(goal_handle, nav_x, nav_y)

        if not path.poses:
            self.get_logger().info("No valid path found. Aborting.")
            goal_handle.abort()
            return FollowPath.Result()

        self.start_process()

        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        self.xy_threshold = goal_handle.request.xy_threshold

        # Steer directly toward the final goal using the continuous control loop
        self.approach_origin = np.array([nav_x, nav_y])
        self.approach_dir = np.array([goal_x, goal_y]) - self.approach_origin
        # self.approach_dir /= np.linalg.norm(self.approach_dir)
        forward_speed = self.default_forward_speed
        self.current_waypoint = (goal_x, goal_y, forward_speed, True)
        self.waypoint_reached = False
        self.reset_pid()

        while True:
            # nav_x, nav_y, _ = self.get_robot_pose()
            # dist = math.sqrt((nav_x - goal_x)**2 + (nav_y - goal_y)**2)

            # change it so that if stationary & reached it switches to normal PID or calls the waypoint server
            if self.waypoint_reached and not goal_handle.request.is_stationary:
                break

            if self.should_abort():
                self.current_waypoint = None
                self.send_stop_cmd()
                self.end_process("Aborted.")
                goal_handle.abort()
                return FollowPath.Result()

            if goal_handle.is_cancel_requested:
                self.current_waypoint = None
                self.send_stop_cmd()
                self.end_process("Canceled.")
                goal_handle.canceled()
                return FollowPath.Result()

            time.sleep(TIMER_PERIOD)

        self.current_waypoint = None
        self.send_stop_cmd()
        self.end_process("FollowPath completed.")
        goal_handle.succeed()
        return FollowPath.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationContinuousServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
