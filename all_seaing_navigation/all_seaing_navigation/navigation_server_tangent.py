#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import CircularPID, PIDController
from all_seaing_interfaces.action import Waypoint, FollowPath
from all_seaing_navigation.planner_executor import PlannerExecutor
from all_seaing_interfaces.msg import ControlOption
from all_seaing_interfaces.msg import ControlOption
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from all_seaing_controller.potential_field import PotentialField

from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseArray, Pose, Point, Vector3, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header, ColorRGBA

from threading import Semaphore, Event
import math
import time

TIMER_PERIOD = 1 / 60

class NavigationTangentServer(ActionServerBase):
    def __init__(self):
        super().__init__("navigation_server_tangent")

        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self.follow_path_callback,
            cancel_callback=self.default_cancel_callback,
            #TODO: refactor cancel logic for navigation
        )
        self.waypoint_client = ActionClient(self, Waypoint, "waypoint")

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/dynamic_map", self.map_callback, 10
        )

        self.avoid_obs = (
            self.declare_parameter("avoid_obs", True)
            .get_parameter_value()
            .bool_value
        )

        self.forward_dist = (
            self.declare_parameter("forward_dist", 5.0)
            .get_parameter_value()
            .double_value
        )

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

        # Pub sub

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
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        # ------- #

        self.map = None

        self.stop_plan_semaphore = Semaphore(1)
        self.stop_plan_evt = Event()

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel

        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel

    def start_plan(self):
        self.stop_plan_evt.set()
        self.stop_plan_semaphore.acquire()
        self.stop_plan_evt.clear()

    def should_abort_plan(self):
        return self.stop_plan_evt.is_set()

    def stopped_plan(self):
        self.stop_plan_semaphore.release()

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = msg

    def visualize_path(self, path: PoseArray):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = self.marker_ns
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.1
        marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        marker_msg.points = [
            Point(x=pose.position.x, y=pose.position.y) for pose in path.poses
        ]
        marker_msg.id = 0
        self.marker_pub.publish(marker_msg)

    def generate_path(self, goal_handle, nav_x, nav_y):
        self.start_plan()  # Protect the long-running generate path function with semaphores

        start = Point(x=nav_x, y=nav_y)
        goal = Point(x=goal_handle.request.x, y=goal_handle.request.y)
        obstacle_tol = goal_handle.request.obstacle_tol
        goal_tol = goal_handle.request.goal_tol

        self.planner = PlannerExecutor(goal_handle.request.planner)
        path = self.planner.plan(self.map, start, goal, obstacle_tol, goal_tol, self.should_abort_plan)
        path.poses = path.poses[(len(path.poses) - 1) % goal_handle.request.choose_every :: goal_handle.request.choose_every]

        self.stopped_plan()  # Release the semaphore
        return path

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

    def dot_3(self, a, b, c):
        return (b[0] - a[0]) * (c[0] - a[0]) + (b[1] - a[1]) * (c[1] - a[1])
    
    def point_segment_scale(self, a, b, c):
        dot_prod = self.dot_3(a, b, c)
        return dot_prod / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]))

    def point_to_segment(self, a, b, c): # point c to segment a-b
        dot_prod = self.dot_3(a, b, c)
        ds_scale = dot_prod / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]))
        if ds_scale < 0 or ds_scale > 1:
            return math.sqrt(min((c[0] - a[0]) * (c[0] - a[0]) + (c[1] - a[1]) * (c[1] - a[1]), (c[0] - b[0]) * (c[0] - b[0]) + (c[1] - b[1]) * (c[1] - b[1])))
        else:
            return math.sqrt((c[0] - a[0]) * (c[0] - a[0]) + (c[1] - a[1]) * (c[1] - a[1]) - ds_scale * dot_prod)

    def proj_vector(self, a, b, c): # point c to line a-b
        dot_prod = self.dot_3(a, b, c)
        sca = dot_prod / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]))
        foot = (a[0] + sca * (b[0] - a[0]), a[1] + sca * (b[1] - a[1]))
        return (foot[0] - c[0], foot[1] - c[1])

    def global_to_robot(self, target, robot):
        rel_pos = (target[0] - robot[0], target[1] - robot[1])
        conv_pos = (rel_pos[0] * math.cos(robot[2]) + rel_pos[1] * math.sin(robot[2]), -rel_pos[0] * math.sin(robot[2]) + rel_pos[1] * math.cos(robot[2]))
        return (conv_pos[0], conv_pos[1], target[2]-robot[2])

    def norm(self, x):
        return math.sqrt(x[0]**2+x[1]**2)
    
    def visualize_waypoint(self, x, y):
        marker_msg = Marker()
        marker_msg.header.frame_id = self.global_frame_id
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "pid_setpoint"
        marker_msg.type = Marker.CYLINDER
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.4
        marker_msg.scale.y = 0.4
        marker_msg.scale.z = 8.0
        marker_msg.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        marker_msg.pose.position.x = x
        marker_msg.pose.position.y = y
        marker_msg.pose.position.z = 2.0
        marker_msg.id = 0
        self.marker_pub.publish(marker_msg)

    def control_loop(self):
        nav_x, nav_y, nav_theta = self.get_robot_pose()
        assert self.cur_seg != None

        if self.cur_seg + 1 >= len(self.path.poses):
            return

        target_pos = (0, 0)

        a = (self.path.poses[self.cur_seg].position.x, self.path.poses[self.cur_seg].position.y)
        b = (self.path.poses[self.cur_seg + 1].position.x, self.path.poses[self.cur_seg + 1].position.y)
        while self.cur_seg + 2 < len(self.path.poses):
            c = (self.path.poses[self.cur_seg + 2].position.x, self.path.poses[self.cur_seg + 2].position.y)
            if self.point_to_segment(a, b, (nav_x, nav_y)) - 1e-9 > self.point_to_segment(b, c, (nav_x, nav_y)):
                self.cur_seg += 1
                a = b
                b = c
            else:
                break

        if self.cur_seg + 2 == len(self.path.poses):
            target_pos = b
        else:
            cor_off = self.proj_vector(a, b, (nav_x, nav_y)) # the perpendicular vector from robot pos to the line

            ds_scale = self.point_segment_scale(a, b, (nav_x, nav_y))

            if ds_scale < 0: # before a
                cor_off = (a[0]-nav_x, a[1]-nav_y)
            elif ds_scale > 1: # after b, shouldn't really happen bc we are considering the closest segment
                cor_off = (b[0]-nav_x, b[1]-nav_y)

            for_sca = self.forward_dist / self.norm((b[0] - a[0], b[1] - a[1]))
            for_off = ((b[0] - a[0]) * for_sca, (b[1] - a[1]) * for_sca) # the tangent vector of the line

            target_pos = (nav_x + cor_off[0] + for_off[0], nav_y + cor_off[1] + for_off[1])

        self.visualize_waypoint(target_pos[0], target_pos[1])
        target_robot_frame = self.global_to_robot((target_pos[0], target_pos[1], math.atan2(b[1]-a[1], b[0]-a[0])), (nav_x, nav_y, nav_theta))
        self.set_pid_setpoints(*target_robot_frame)
        self.update_pid(0, 0, 0)

        x_vel = self.x_pid.get_effort()
        y_vel = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()

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

    def get_closest_seg(self, nav_x, nav_y):
        opt = (1000000, 0)
        for i in range(0, len(self.path.poses) - 1):
            a = (self.path.poses[i].position.x, self.path.poses[i].position.y)
            b = (self.path.poses[i + 1].position.x, self.path.poses[i + 1].position.y)
            opt = min(opt, (self.point_to_segment(a, b, (nav_x, nav_y)), i))
        return opt[1]


    def follow_path_callback(self, goal_handle):
        self.get_logger().info("Path following started!")

        # Immediately start and end process if no map is found
        if self.map is None:
            self.get_logger().info("No valid path found. Aborting path following.")
            goal_handle.abort()
            return FollowPath.Result()

        # Get robot pose
        nav_x, nav_y, _ = self.get_robot_pose()

        # Generate path using requested planner
        self.path = self.generate_path(goal_handle, nav_x, nav_y)
        if not self.path.poses:
            self.get_logger().info("No valid path found. Aborting path following.")
            goal_handle.abort()
            return FollowPath.Result()

        self.start_process()

        self.visualize_path(self.path)

        self.cur_seg = self.get_closest_seg(nav_x, nav_y)

        self.reset_pid()

        while True:
            nav_x, nav_y, _ = self.get_robot_pose()
            if (nav_x - goal_handle.request.x) * (nav_x - goal_handle.request.x) + (nav_y - goal_handle.request.y) * (nav_y - goal_handle.request.y) < goal_handle.request.xy_threshold and not goal_handle.request.is_stationary:
                break

            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return FollowPath.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Path following canceled!")
                goal_handle.canceled()
                return FollowPath.Result()

            self.control_loop()

            time.sleep(TIMER_PERIOD)
            
        self.end_process("Path following completed!")
        goal_handle.succeed()
        return FollowPath.Result(is_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTangentServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
