#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data


from all_seaing_interfaces.action import Task
from all_seaing_controller.pid_controller import PIDController, CircularPID
from ament_index_python.packages import get_package_share_directory
from all_seaing_interfaces.msg import ControlOption, LabeledObjectPlane, LabeledObjectPlaneArray
from all_seaing_common.task_server_base import TaskServerBase
from sensor_msgs.msg import CameraInfo
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from all_seaing_autonomy.roboboat.visualization_tools import VisualizationTools

from dataclasses import dataclass

import os
import yaml
import time
import math
import numpy as np

class DeliveryState(Enum):
    WAITING_TARGET = 1
    NAVIGATING_TARGET = 2
    STATIONKEEPING = 3
    CANCELLING_NAVIGATION = 4
    NEW_NAVIGATION = 5

class TargetType(Enum):
    def __str__(self):
        return str(self.name)
    WATER_TARGET = 1
    BALL_TARGET = 2

class MechanismNavigation(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "mechanism_navigation", action_name = "mechanism_navigation", search_action_name = "search_delivery")

        self.obj_plane_sub = self.create_subscription(
            LabeledObjectPlaneArray, "object_planes/global", self.plane_cb, qos_profile_sensor_data
        )

        self.delivery_marker_pub = self.create_publisher(MarkerArray, 'delivery_marker_pub', 10)

        self.controller_marker_pub = self.create_publisher(
            MarkerArray, "controller_markers", 10
        )

        self.declare_parameter("forward_speed", 2.0)
        self.declare_parameter("max_yaw", 0.7)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.declare_parameter("slow_dist", 0.1) # larger->smoother decline, starts slowing down further away
        self.slow_dist = self.get_parameter("slow_dist").get_parameter_value().double_value

        self.declare_parameter("navigation_dist_thres", 7.0)
        self.navigation_dist_thres = self.get_parameter("navigation_dist_thres").get_parameter_value().double_value

        self.declare_parameter("update_target_dist_thres", 3.0)
        self.update_target_dist_thres = self.get_parameter("update_target_dist_thres").get_parameter_value().double_value

        self.declare_parameter("adapt_dist", 0.7)
        self.adapt_dist = self.get_parameter("adapt_dist").get_parameter_value().double_value

        self.declare_parameter("wpt_banner_dist", 4.0)
        self.wpt_banner_dist = self.get_parameter("wpt_banner_dist").get_parameter_value().double_value

        self.declare_parameter("shooting_xy_thres", 0.2)
        self.shooting_xy_thres = self.get_parameter("shooting_xy_thres").get_parameter_value().double_value

        self.declare_parameter("shooting_theta_thres", 10.0)
        self.shooting_theta_thres = self.get_parameter("shooting_theta_thres").get_parameter_value().double_value

        self.declare_parameter("duplicate_dist", 0.5)
        self.duplicate_dist = self.get_parameter("duplicate_dist").get_parameter_value().double_value

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
            self.declare_parameter("max_vel", [2.0, 1.0, 0.4])
            .get_parameter_value()
            .double_array_value
        )

        self.vel_marker_scale = (
            self.declare_parameter("vel_marker_scale", 1.0)
            .get_parameter_value()
            .double_value
        )

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        self.time_last_had_target = time.time()
        self.time_started_shooting = -1

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter(
            "shape_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "shape_label_mappings.yaml"
            ),
        )

        shape_label_mappings_file = self.get_parameter(
            "shape_label_mappings_file"
        ).value
        with open(shape_label_mappings_file, "r") as f:
            self.label_mappings = yaml.safe_load(f)

        if self.is_sim:
            self.water_labels = [self.label_mappings[name] for name in ["blue_circle", "blue_cross", "blue_triangle", "green_circle", "green_cross", "green_square", "green_triangle"]]
            self.ball_labels = [self.label_mappings[name] for name in ["red_circle", "red_cross", "red_triangle", "red_square"]]
        else:
            self.water_labels = [self.label_mappings[name] for name in ["black_triangle"]]
            self.ball_labels = [self.label_mappings[name] for name in ["black_cross"]]

        # update from subs
        self.water_banners = []
        self.ball_banners = []
        self.selected_target = None
        self.shot_targets = []
        self.plane_msg = None

        self.got_target = False
        self.picked_target = False
        self.started_task = False

        self.state = DeliveryState.WAITING_TARGET

        self.shot_water = False
        self.shot_ball = False

    def ctr_normal(self, plane: LabeledObjectPlane):
        center_pt = (plane.normal_ctr.position.x, plane.normal_ctr.position.y)
        _,_,theta = euler_from_quaternion([plane.normal_ctr.orientation.x, plane.normal_ctr.orientation.y, plane.normal_ctr.orientation.z, plane.normal_ctr.orientation.w])
        return (center_pt, (np.cos(theta), np.sin(theta)))

    def plane_cb(self, msg: LabeledObjectPlaneArray):
        self.plane_msg = msg
        if not self.started_task:
            return
        self.find_target()
        
    def find_target(self):
        if self.plane_msg is None:
            return False
        # self.get_logger().info('GOT OBJECTS')
        self.got_target = False
        self.picked_target = False
        self.updated_target_pos = False
        new_water_banners = []
        new_ball_banners = []
        obj_plane: LabeledObjectPlane
        for obj_plane in self.plane_msg.objects:
            if(obj_plane.label in self.water_labels):
                self.got_target = True
                ctr, normal = self.ctr_normal(obj_plane)
                new_water_banners.append((TargetType.WATER_TARGET, (ctr, normal)))
            if (obj_plane.label in self.ball_labels):
                self.got_target = True
                ctr, normal = self.ctr_normal(obj_plane)
                new_ball_banners.append((TargetType.BALL_TARGET, (ctr, normal)))
        
        # update global variables
        self.water_banners = new_water_banners
        self.ball_banners = new_ball_banners

        # ensure that we first shoot two targets of different type
        if self.shot_water and not self.shot_ball and len(self.ball_banners) > 0:
            target_banners = self.ball_banners
        elif self.shot_ball and not self.shot_water and len(self.water_banners) > 0:
            target_banners = self.water_banners
        else:
            target_banners = self.water_banners + self.ball_banners

        if self.got_target:
            # check for taken docks and stuff
            for target_type, (target_ctr, target_normal) in target_banners:
                # check if we already shot it
                shot = False
                for shot_target_type, (shot_target_ctr, shot_target_normal) in self.shot_targets:
                    if self.norm(shot_target_ctr, target_ctr) < self.duplicate_dist:
                        shot = True
                if shot:
                    continue
                # found new docking target
                self.picked_target = True
                if self.selected_target is None:
                    self.updated_target_pos = True
                if (self.selected_target is not None) and (self.norm(self.selected_target[1][0], target_ctr) < self.duplicate_dist):
                    # same target, update position & normal
                    self.selected_target = (target_type, (target_ctr, target_normal))
                    self.updated_target_pos = True
                if (self.selected_target is None) or (self.norm(self.selected_target[1][0], self.robot_pos) > self.norm(target_ctr, self.robot_pos) + self.update_target_dist_thres):
                    self.state = DeliveryState.NEW_NAVIGATION
                    # found a new one closer
                    self.selected_target = (target_type, (target_ctr, target_normal))
                    self.updated_target_pos = True
                    # self.pid.reset()
                    self.x_pid.reset()
                    self.y_pid.reset()
                    self.theta_pid.reset()
                    self.get_logger().info(f'WILL SHOOT {self.selected_target[0]}')

        if (not self.picked_target) or (not self.updated_target_pos):
        # if (not self.picked_target):
            self.selected_target = None
            self.picked_target = False
            # self.pid.reset()
            self.x_pid.reset()
            self.y_pid.reset()
            self.theta_pid.reset()
            if self.state == DeliveryState.NAVIGATING_TARGET:
                # was going to a fake target (misdetection)
                self.state = DeliveryState.CANCELLING_NAVIGATION
                self.get_logger().info(f'WAS GOING TO A FAKE TARGET, ABORT')
            else:
                self.state = DeliveryState.WAITING_TARGET
            return False
        else:
            return True
        
        # if not self.updated_target_pos:
        #     self.selected_target = None
        #     self.picked_target = False
        #     self.get_logger().info("COULDN'T TRACK TARGET POSITION")
        #     self.x_pid.reset()
        #     self.y_pid.reset()
        #     self.theta_pid.reset()
    
    def distance(self, point, wall_params):
        x, y = point
        (a, b, c), _ = wall_params
        return (a*x+b+y)/math.sqrt(a**2+b**2)
    
    def dot(self, vec1, vec2):
        return vec1[0]*vec2[0]+vec1[1]*vec2[1]

    def midpoint(self, pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        return ((x1+x2)/2.0, (y1+y2)/2.0)

    def difference(self, pt1, pt2):
        """
        pt2 - pt1
        """
        x1, y1 = pt1
        x2, y2 = pt2
        return (x2-x1, y2-y1)
    
    def norm_squared(self, vec, ref=(0, 0)):
        return (vec[0] - ref[0])**2 + (vec[1]-ref[1])**2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))
    
    def perp_vec(self, vec):
        return (-vec[1], vec[0])
    
    # def angle_vec(self, vec1, vec2):
    #     return math.acos(self.dot(vec1, vec2)/(self.norm(vec1)*self.norm(vec2))) 
    
    def negative(self, vec):
        return (-vec[0], -vec[1])
    
    def sum(self, vec1, vec2):
        return (vec1[0]+vec2[0], vec1[1]+vec2[1])
    
    def scalar_prod(self, vec, scalar):
        return (scalar*vec[0], scalar*vec[1])
    
    def cross(self, vec1, vec2):
        return vec1[0]*vec2[1]-vec1[1]*vec2[0]

    def angle_vec(self, vec1, vec2):
        return math.atan2(self.cross(vec1, vec2)/(self.norm(vec1)*self.norm(vec2)), self.dot(vec1, vec2)/(self.norm(vec1)*self.norm(vec2))) 

    def angle_segments(self, p1, p2):
        p1left, p1right = p1
        p2left, p2right = p2
        return self.angle_vec(self.difference(p1left, p1right), self.difference(p2left, p2right))
    
    def fit_pair(self, point_pair):
        (x1,y1), (x2,y2) = point_pair
        # (y-y1)(x2-x1) = (y2-y1)*(x-x1) -> y(x2-x1)+x(y1-y2)+x1(y2-y1)+y1(x1-x2) = 0 -> y(x2-x1)+x(y1-y2)+x1y2-x2y1
        return (y1-y2, x2-x1, x1*y2-x2*y1), abs(x1*y2-x2*y1)/math.sqrt((y1-y2)**2+(x2-x1)**2)
    
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
    
    def should_accept_task(self, goal_request):
        if self.state == DeliveryState.WAITING_TARGET:
            return self.find_target()
        else:
            return True
    
    def init_setup(self):
        self.started_task = True
        self.time_last_had_target = time.time()
        self.set_pid_setpoints(0, 0, 0)
        self.mark_successful()

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
    
    def control_loop(self):
        if self.state == DeliveryState.WAITING_TARGET:
            # IF DON'T HAVE A TARGET FOR TOO LONG, FINISH TASK
            if time.time() - self.time_last_had_target > 5:
                self.mark_successful()
            return # TODO stationkeep/search for target by steering right and left
        self.time_last_had_target = time.time()
        if self.state == DeliveryState.CANCELLING_NAVIGATION:
            self.cancel_navigation()
            self.state = DeliveryState.WAITING_TARGET
            return
        marker_arr = MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)])
        mark_id = 1
        # self.get_logger().info(f'CONTROL LOOP')
        # PID to go to the detected target (consider its middle and the angle of the whole dock line)
        target_back_mid = self.selected_target[1][0]
        target_dir = self.selected_target[1][1]
        
        marker_arr.markers.append(VisualizationTools.visualize_line(target_back_mid, self.perp_vec(target_dir), mark_id, (0.0, 0.0, 1.0), self.robot_frame_id))
        mark_id = mark_id + 1

        if self.state == DeliveryState.STATIONKEEPING or self.norm(target_back_mid, self.robot_pos) < self.navigation_dist_thres:
            if self.state == DeliveryState.NAVIGATING_TARGET:
                self.get_logger().info('CANCELLING NAVIGATION')
                self.cancel_navigation()
            self.get_logger().info('STATIONKEEPING PID')
            self.state = DeliveryState.STATIONKEEPING
            # go to that line and forward (negative error if boat left of line, positive if right)
            offset = -self.dot(self.difference(target_back_mid, self.robot_pos), self.perp_vec(target_dir))
            approach_angle = self.angle_vec(self.negative(target_dir), self.robot_dir) # TODO Check sign
            angle_error = self.angle_vec(self.difference(self.robot_pos, target_back_mid), self.robot_dir)

            # forward speed decreasing exponentially as we get closer
            dist_diff = self.dot(self.difference(target_back_mid, self.robot_pos), target_dir) - self.wpt_banner_dist
            # forward_speed = self.forward_speed*(1-np.exp(-dist_diff/self.slow_dist))

            self.get_logger().info(f'side offset: {offset}')
            self.get_logger().info(f'forward distance: {dist_diff}')
            self.update_pid(-dist_diff, offset, angle_error) # could also use PID for the x coordinate, instead of the exponential thing we did above
            if abs(offset) < self.shooting_xy_thres and abs(dist_diff) < self.shooting_xy_thres and abs(angle_error) < self.shooting_theta_thres/180.0*np.pi:
                # TODO SHOOT BALL/WATER
                # self.get_logger().info(f'SHOOTING {self.selected_target[0]}')

                if self.time_started_shooting == -1:
                    self.get_logger().info(f'STARTED SHOOTING {self.selected_target[0]}, TIME: {time.time()}')
                    self.time_started_shooting = time.time()
                    return
                elif time.time() - self.time_started_shooting > 5:
                    self.get_logger().info(f'SHOT {self.selected_target[0]}, TIME: {time.time()}')
                    # move on
                    self.shot_targets.append(self.selected_target)

                    if self.selected_target[0] == TargetType.WATER_TARGET:
                        self.shot_water = True
                    if self.selected_target[0] == TargetType.BALL_TARGET:
                        self.shot_ball = True

                    self.state = DeliveryState.WAITING_TARGET
                    self.selected_target = None
                    self.picked_target = False
                    # self.pid.reset()
                    self.x_pid.reset()
                    self.y_pid.reset()
                    self.theta_pid.reset()
                    self.time_started_shooting = -1
                    self.send_vel_cmd(0.0,0.0,0.0)
                    return
            x_output = self.x_pid.get_effort()
            y_output = self.y_pid.get_effort()
            theta_output = self.theta_pid.get_effort()
            x_vel = x_output*np.cos(approach_angle) + y_output*np.sin(approach_angle)
            # x_vel = forward_speed
            y_vel = y_output*np.cos(approach_angle) - x_output*np.sin(approach_angle)

            marker_array = MarkerArray()
            marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 1.0, 0.0), id=0))

            marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 0.0, 1.0), id=2))

            x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
            self.send_vel_cmd(x_vel, y_vel, theta_output)
            self.controller_marker_pub.publish(marker_array)
        else:
            waypoint = self.sum(target_back_mid, self.scalar_prod(target_dir, self.wpt_banner_dist))
            if self.state == DeliveryState.NEW_NAVIGATION or ((self.sent_waypoint is not None) and (self.norm(waypoint, self.sent_waypoint) > self.adapt_dist)):
                self.get_logger().info('SENDING WAYPOINT')
                # self.get_logger().info(f'passed: {passed_previous}, first passed: {passed_previous}, first buoy pair: {self.first_buoy_pair}, changed pair to: {changed_pair_to}, adapt waypoint: {adapt_waypoint}')
                self.send_waypoint_to_server(waypoint)
                self.state = DeliveryState.NAVIGATING_TARGET
            elif self.send_goal_future != None and self.sent_waypoint != None:
                goal_result = self.send_goal_future.result()
                if self.waypoint_rejected or self.waypoint_aborted:
                    # follow path failed, retry sending
                    self.get_logger().info("Waypoint request aborted by nav server and no new waypoint option found. Resending request...")
                    self.send_waypoint_to_server(self.sent_waypoint)
            
        self.delivery_marker_pub.publish(marker_arr)


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
