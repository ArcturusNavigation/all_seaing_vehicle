#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA

from ament_index_python.packages import get_package_share_directory

from all_seaing_controller.pid_controller import PIDController, CircularPID
from all_seaing_interfaces.msg import LabeledObjectPlaneArray, LabeledObjectPlane
from all_seaing_controller.potential_field import PotentialField
from all_seaing_common.task_server_base import TaskServerBase

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from all_seaing_autonomy.roboboat.visualization_tools import VisualizationTools

import time
import math
import yaml
import os
import numpy as np
import random
from enum import Enum
from action_msgs.msg import GoalStatus

class DockingState(Enum):
    WAITING_DOCK = 1
    NAVIGATING_DOCK = 2
    DOCKING = 3
    CANCELLING_NAVIGATION = 4
    NEW_NAVIGATION = 5

class Docking(TaskServerBase):
    def __init__(self):
        super().__init__(server_name = "docking_server", action_name = "docking", search_action_name = "search_docking")

        self.obj_plane_sub = self.create_subscription(
            LabeledObjectPlaneArray, "object_planes/global", self.plane_cb, qos_profile_sensor_data
        )

        self.docking_marker_pub = self.create_publisher(MarkerArray, 'docking_marker_pub', 10)

        self.declare_parameter("forward_speed", 2.0)
        self.declare_parameter("max_yaw", 0.7)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.declare_parameter("dock_width", 1.0)
        self.declare_parameter("dock_length", 3.0)
        self.dock_width = self.get_parameter("dock_width").get_parameter_value().double_value
        self.dock_length = self.get_parameter("dock_length").get_parameter_value().double_value

        self.declare_parameter("boat_dock_dist_thres", 5.0)
        self.boat_dock_dist_thres = self.get_parameter("boat_dock_dist_thres").get_parameter_value().double_value

        self.declare_parameter("boat_taken_angle_thres", 0.8) # angle of banner-boat to dock segment over pi/2
        self.boat_taken_angle_thres = self.get_parameter("boat_taken_angle_thres").get_parameter_value().double_value

        self.declare_parameter("boat_angle_coeff", 0.8)
        self.boat_angle_coeff = self.get_parameter("boat_angle_coeff").get_parameter_value().double_value

        self.declare_parameter("dock_merged_id", 100)
        self.dock_merged_id = self.get_parameter("dock_merged_id").get_parameter_value().integer_value

        self.declare_parameter("slow_dist", 0.1) # larger->smoother decline, starts slowing down further away
        self.slow_dist = self.get_parameter("slow_dist").get_parameter_value().double_value

        self.declare_parameter("navigation_dist_thres", 7.0)
        self.navigation_dist_thres = self.get_parameter("navigation_dist_thres").get_parameter_value().double_value

        self.declare_parameter("update_slot_dist_thres", 3.0)
        self.update_slot_dist_thres = self.get_parameter("update_slot_dist_thres").get_parameter_value().double_value

        self.declare_parameter("adapt_dist", 0.7)
        self.adapt_dist = self.get_parameter("adapt_dist").get_parameter_value().double_value

        self.declare_parameter("wpt_banner_dist", 4.0)
        self.wpt_banner_dist = self.get_parameter("wpt_banner_dist").get_parameter_value().double_value

        self.declare_parameter("docked_xy_thres", 0.2)
        self.docked_xy_thres = self.get_parameter("docked_xy_thres").get_parameter_value().double_value

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

        self.avoid_obs = (
            self.declare_parameter("avoid_obs", True)
            .get_parameter_value()
            .bool_value
        )

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)

        # for obstacle avoidance
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "/point_cloud/filtered_obs", self.point_cloud_cb, qos_profile_sensor_data
        )

        self.lidar_point_cloud = None

        self.controller_marker_pub = self.create_publisher(
            MarkerArray, "controller_markers", 10
        )
        
        # update from subs
        self.dock_banners = []
        self.dock_banner_line = None
        self.boat_banners = []
        self.selected_slot = None
        self.taken = []
        self.plane_msg = None

        self.got_dock = False
        self.picked_slot = False
        self.started_task = False

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
            self.dock_labels = [self.label_mappings[name] for name in ["blue_circle", "blue_cross", "blue_triangle", "green_circle", "green_cross", "green_square", "green_triangle", "red_circle", "red_cross", "red_triangle", "red_square"]]
            self.boat_labels = [self.label_mappings[name] for name in ["black_cross", "black_triangle"]]
        else:
            # TODO replace w/ numbers for the dock banner labels, as in the roboboat course
            # self.dock_labels = [self.label_mappings[name] for name in ["blue_circle", "blue_cross", "blue_triangle", "green_circle", "green_cross", "green_square", "green_triangle", "red_circle", "red_cross", "red_triangle", "red_square"]]
            # self.boat_labels = [self.label_mappings[name] for name in ["black_cross", "black_triangle"]]
            self.dock_labels = [self.label_mappings[name] for name in ["black_cross", "black_triangle"]]
            self.boat_labels = []
        
        self.inv_label_mappings = {}
        for key, value in self.label_mappings.items():
            self.inv_label_mappings[value] = key

        self.state = DockingState.WAITING_DOCK

    def point_cloud_cb(self, msg):
        self.lidar_point_cloud = msg

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

    def ctr_normal(self, plane: LabeledObjectPlane):
        center_pt = (plane.normal_ctr.position.x, plane.normal_ctr.position.y)
        _,_,theta = euler_from_quaternion([plane.normal_ctr.orientation.x, plane.normal_ctr.orientation.y, plane.normal_ctr.orientation.z, plane.normal_ctr.orientation.w])
        return (center_pt, (np.cos(theta), np.sin(theta)))

    def plane_cb(self, msg: LabeledObjectPlaneArray):
        self.plane_msg = msg
        if not self.started_task:
            return
        self.find_docking_slot()
        
    def find_docking_slot(self):
        if self.plane_msg is None:
            return False
        # self.get_logger().info('GOT OBJECTS')
        self.got_dock = False
        self.picked_slot = False
        self.updated_slot_pos = False
        new_dock_banners = []
        new_boat_banners = []
        obj_plane: LabeledObjectPlane
        for obj_plane in self.plane_msg.objects:
            if(obj_plane.label in self.boat_labels):
                ctr, normal = self.ctr_normal(obj_plane)
                new_boat_banners.append((obj_plane.label, (ctr, normal)))
                # self.get_logger().info(f'BOAT: {self.inv_label_mappings[obj_plane.label]} -> {(ctr, normal).__str__()}')
            if (obj_plane.label in self.dock_labels):
                self.got_dock = True
                ctr, normal = self.ctr_normal(obj_plane)
                new_dock_banners.append((obj_plane.label, (ctr, normal)))
                # self.get_logger().info(f'SLOT: {self.inv_label_mappings[obj_plane.label]} -> {(ctr, normal).__str__()}')
        
        # update global variables
        self.dock_banners = new_dock_banners
        self.boat_banners = new_boat_banners

        # self.get_logger().info(f'GOT {len(self.dock_banners)} SLOTS AND {len(self.boat_banners)} BOATS')

        if self.got_dock:
            # check for taken docks and stuff
            for dock_label, (dock_ctr, dock_normal) in self.dock_banners:
                if (dock_label in self.taken):
                    continue # just ignore, since we saw that it was taken once it's always taken
                # check if any boat is in that slot
                taken = False
                for boat_label, (boat_ctr, boat_normal) in self.boat_banners:
                    # check if boat is closer than dock and angle of dock and boat banners is relatively perpendicular to the dock
                    if self.norm(boat_ctr, dock_ctr) < self.boat_dock_dist_thres and abs(self.angle_vec(self.difference(dock_ctr, boat_ctr), dock_normal)) < self.boat_taken_angle_thres:
                        # taken
                        taken = True
                if taken:
                    self.taken.append(dock_label)
                    if(self.picked_slot and self.selected_slot[0] == dock_label):
                        # we're cooked
                        self.selected_slot = None
                        self.picked_slot = False
                        # self.pid.reset()
                        self.x_pid.reset()
                        self.y_pid.reset()
                        self.theta_pid.reset()
                        if self.state == DockingState.NAVIGATING_DOCK:
                            self.state = DockingState.CANCELLING_NAVIGATION
                        else:
                            self.state = DockingState.WAITING_DOCK
                        self.get_logger().info(f'DOCK {self.inv_label_mappings[dock_label]} IS TAKEN')
                    continue
                # found empty docking slot
                self.picked_slot = True
                if self.selected_slot is None:
                    self.updated_slot_pos = True
                if (self.selected_slot is not None) and (self.selected_slot[0] == dock_label) and (self.norm(self.selected_slot[1][0], dock_ctr) < self.duplicate_dist):
                    # same slot, update position & normal
                    self.selected_slot = (dock_label, (dock_ctr, dock_normal))
                    self.updated_slot_pos = True
                if (self.selected_slot is None) or (self.norm(self.selected_slot[1][0], self.robot_pos) > self.norm(dock_ctr, self.robot_pos) + self.update_slot_dist_thres):
                    self.state = DockingState.NEW_NAVIGATION
                    # found an empty one closer
                    self.selected_slot = (dock_label, (dock_ctr, dock_normal))
                    self.updated_slot_pos = True
                    # self.pid.reset()
                    self.x_pid.reset()
                    self.y_pid.reset()
                    self.theta_pid.reset()
                    self.get_logger().info(f'WILL DOCK INTO {self.inv_label_mappings[self.selected_slot[0]]}')

        if (not self.picked_slot) or (not self.updated_slot_pos):
        # if (not self.picked_slot):
            self.selected_slot = None
            self.picked_slot = False
            # self.pid.reset()
            self.x_pid.reset()
            self.y_pid.reset()
            self.theta_pid.reset()
            if self.state == DockingState.NAVIGATING_DOCK:
                # was going to a fake slot (misdetection)
                self.state = DockingState.CANCELLING_NAVIGATION
                self.get_logger().info(f'WAS GOING TO A FAKE DOCK, ABORT')
            else:
                self.state = DockingState.WAITING_DOCK
            return False
        else:
            return True
        
        # if not self.updated_slot_pos:
        #     self.selected_slot = None
        #     self.picked_slot = False
        #     self.get_logger().info("COULDN'T TRACK SLOT POSITION")
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
        if self.state == DockingState.WAITING_DOCK:
            return self.find_docking_slot()
        else:
            return True

    def init_setup(self):
        self.started_task = True
        self.set_pid_setpoints(0, 0, 0)
        self.mark_successful()
    
    def control_loop(self):
        if self.state == DockingState.WAITING_DOCK:
            return # TODO stationkeep/search for dock by steering right and left
        if self.state == DockingState.CANCELLING_NAVIGATION:
            self.cancel_navigation()
            self.state = DockingState.WAITING_DOCK
            return
        marker_arr = MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)])
        mark_id = 1
        # self.get_logger().info(f'CONTROL LOOP')
        # PID to go to the detected slot (consider its middle and the angle of the whole dock line)
        slot_back_mid = self.selected_slot[1][0]
        slot_dir = self.selected_slot[1][1]
        
        marker_arr.markers.append(VisualizationTools.visualize_line(slot_back_mid, self.perp_vec(slot_dir), mark_id, (0.0, 0.0, 1.0), self.robot_frame_id))
        mark_id = mark_id + 1

        if self.state == DockingState.DOCKING or self.norm(slot_back_mid, self.robot_pos) < self.navigation_dist_thres:
            if self.state == DockingState.NAVIGATING_DOCK:
                self.get_logger().info('CANCELLING NAVIGATION')
                self.cancel_navigation()
            self.get_logger().info('DOCKING PID')
            self.state = DockingState.DOCKING
            # go to that line and forward (negative error if boat left of line, positive if right)
            offset = -self.dot(self.difference(slot_back_mid, self.robot_pos), self.perp_vec(slot_dir))
            approach_angle = self.angle_vec(self.negative(slot_dir), self.robot_dir) # TODO Check sign

            # dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
            # self.pid.update(offset + self.boat_angle_coeff*approach_angle, dt)            
            # yaw_rate = self.pid.get_effort()
            # self.prev_update_time = self.get_clock().now()

            # control_msg = ControlOption()
            # control_msg.priority = 1

            # forward speed decreasing exponentially as we get closer
            dist_diff = self.dot(self.difference(slot_back_mid, self.robot_pos), slot_dir) - self.dock_length/2.0
            # subtract half the dock length when further than the half the width sideways
            if abs(offset) > self.dock_width/2.0:
                dist_diff -= self.dock_length/2.0
            # forward_speed = self.forward_speed*(1-np.exp(-dist_diff/self.slow_dist))

            # control_msg.twist.linear.x = float(forward_speed)
            # control_msg.twist.linear.y = 0.0
            # control_msg.twist.angular.z = float(yaw_rate)
            # self.control_pub.publish(control_msg)

            self.get_logger().info(f'side offset: {offset}')
            self.get_logger().info(f'forward distance: {dist_diff}')
            self.update_pid(-dist_diff, offset, approach_angle) # could also use PID for the x coordinate, instead of the exponential thing we did above
            if abs(offset) < self.docked_xy_thres and abs(dist_diff) < self.docked_xy_thres:
                self.send_vel_cmd(0.0,0.0,0.0)
                self.mark_successful()
                return
            x_output = self.x_pid.get_effort()
            y_output = self.y_pid.get_effort()
            theta_output = self.theta_pid.get_effort()
            x_vel = x_output*np.cos(approach_angle) + y_output*np.sin(approach_angle)
            # x_vel = forward_speed
            y_vel = y_output*np.cos(approach_angle) - x_output*np.sin(approach_angle)

            marker_array = MarkerArray()
            marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 1.0, 0.0), id=0))

            # obstacle avoidance
            avoid_x_vel, avoid_y_vel = 0.0, 0.0
            if self.avoid_obs and (self.lidar_point_cloud is not None) and (self.lidar_point_cloud.width > 0):
                avoid_x_vel, avoid_y_vel = PotentialField(self.lidar_point_cloud, self.avoid_max_dist).sketchy_gradient_descent_step()
                # self.get_logger().info(f'{self.lidar_point_cloud.width} points, unscaled avoiding vel: {avoid_x_vel, avoid_y_vel}')
                avoid_x_vel *= self.avoid_vel_coeff
                avoid_y_vel *= self.avoid_vel_coeff
                # rotational avoidance velocity
                rot_avoid_x_vel, rot_avoid_y_vel = PotentialField(self.lidar_point_cloud, self.avoid_max_dist).rotational_force(vel_dir=(x_vel, y_vel))
                # self.get_logger().info(f'{self.lidar_point_cloud.width} points, unscaled avoiding vel: {avoid_x_vel, avoid_y_vel}')
                rot_avoid_x_vel *= self.rot_avoid_vel_coeff
                rot_avoid_y_vel *= self.rot_avoid_vel_coeff
                if self.avoid_rot_vel_mag:
                    vel_mag = math.sqrt(x_vel**2+y_vel**2)
                    rot_avoid_x_vel*=vel_mag
                    rot_avoid_y_vel*=vel_mag
                avoid_x_vel += rot_avoid_x_vel
                avoid_y_vel += rot_avoid_y_vel
                x_vel += avoid_x_vel
                y_vel += avoid_y_vel

            marker_array.markers.append(self.vel_to_marker((avoid_x_vel, avoid_y_vel), scale=self.vel_marker_scale, rgb=(1.0, 0.0, 0.0), id=1))
            marker_array.markers.append(self.vel_to_marker((x_vel, y_vel), scale=self.vel_marker_scale, rgb=(0.0, 0.0, 1.0), id=2))

            x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
            self.send_vel_cmd(x_vel, y_vel, theta_output)
            self.controller_marker_pub.publish(marker_array)
        else:
            waypoint = self.sum(slot_back_mid, self.scalar_prod(slot_dir, self.wpt_banner_dist))
            if self.state == DockingState.NEW_NAVIGATION or ((self.sent_waypoint is not None) and (self.norm(waypoint, self.sent_waypoint) > self.adapt_dist)):
                self.get_logger().info('SENDING WAYPOINT')
                # self.get_logger().info(f'passed: {passed_previous}, first passed: {passed_previous}, first buoy pair: {self.first_buoy_pair}, changed pair to: {changed_pair_to}, adapt waypoint: {adapt_waypoint}')
                self.send_waypoint_to_server(waypoint)
                self.state = DockingState.NAVIGATING_DOCK
            elif self.send_goal_future != None and self.sent_waypoint != None:
                goal_result = self.send_goal_future.result()
                if self.waypoint_rejected or self.waypoint_aborted:
                    # follow path failed, retry sending
                    self.get_logger().info("Waypoint request aborted by nav server and no new waypoint option found. Resending request...")
                    self.send_waypoint_to_server(self.sent_waypoint)
            
        self.docking_marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    node = Docking()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()