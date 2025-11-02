#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController, CircularPID
from all_seaing_interfaces.action import FollowPath, Task, Waypoint
from all_seaing_interfaces.msg import LabeledObjectPlaneArray, LabeledObjectPlane, ControlOption

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

class Docking(ActionServerBase):
    def __init__(self):
        super().__init__("docking_server")

        self._action_server = ActionServer(
            self,
            Task,
            "docking",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        self.control_pub = self.create_publisher(
            ControlOption, 
            "control_options", 
            10
        )

        self.obj_plane_sub = self.create_subscription(
            LabeledObjectPlaneArray, "object_planes/global", self.plane_cb, qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'docking_marker_pub', 10)

        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        
        self.declare_parameter("xy_threshold", 1.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("wpt_theta_threshold", 10.0)
        self.declare_parameter("goal_tol", 1.0)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 10)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")
        self.declare_parameter("bypass_planner", False)

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

        self.declare_parameter("timer_period", 1/30.0)
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

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

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)
        
        # update from subs
        self.dock_banners = []
        self.dock_banner_line = None
        self.boat_banners = []
        self.selected_slot = None
        self.taken = []

        self.got_dock = False
        self.picked_slot = False
        self.started_task = False

        self.state = None
        self.result = False

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
        
        self.dock_labels = [self.label_mappings[name] for name in ["blue_circle", "blue_cross", "blue_triangle", "green_circle", "green_cross", "green_square", "green_triangle", "red_circle", "red_cross", "red_triangle", "red_square"]]
        self.boat_labels = [self.label_mappings[name] for name in ["black_circle", "black_cross", "black_triangle"]]
        self.inv_label_mappings = {}
        for key, value in self.label_mappings.items():
            self.inv_label_mappings[value] = key

        self.waypoint_sent_future = None
        self.send_goal_future = None

        self.state = DockingState.WAITING_DOCK

        self.lastSelectedGoal = None

        self.waypoint_reject = False
        self.waypoint_done = False

        self.sent_waypoint = None

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
        return (math.cos(heading), math.sin(heading))

    def _waypoint_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Strange - sent waypoint rejected immediately.")
            self.waypoint_reject = True
            return
        self.waypoint_done = True
        self.waypoint_sent_future = goal_handle.get_result_async()

    def send_waypoint_to_server(self, waypoint):
        # self.get_logger().info('SENDING WAYPOINT TO SERVER')
        # sending waypoints to navigation server
        self.waypoint_sent_future = None # Reset this... Make sure chance of going backwards is 0
        self.waypoint_reject = False
        self.waypoint_done = False

        self.sent_waypoint = waypoint
        if not self.bypass_planner:
            self.follow_path_client.wait_for_server()
            goal_msg = FollowPath.Goal()
            goal_msg.planner = self.get_parameter("planner").value
            goal_msg.x = waypoint[0]
            goal_msg.y = waypoint[1]
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
            goal_msg.goal_tol = self.get_parameter("goal_tol").value
            goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
            goal_msg.choose_every = self.get_parameter("choose_every").value
            goal_msg.is_stationary = True
            self.follow_path_client.wait_for_server()
            self.send_goal_future = self.follow_path_client.send_goal_async(
                goal_msg
            )
            self.send_goal_future.add_done_callback(self._waypoint_sent_callback)
        else:
            goal_msg = Waypoint.Goal()
            goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
            goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
            goal_msg.x = waypoint[0]
            goal_msg.y = waypoint[1]
            goal_msg.ignore_theta = True
            goal_msg.is_stationary = True
            self.result = False
            self.waypoint_client.wait_for_server()
            self.send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self._waypoint_sent_callback)
        self.lastSelectedGoal = waypoint
    
    def cancel_navigation(self):
        if not self.waypoint_done:
            while (self.send_goal_future is not None) and (not self.send_goal_future.cancelled()):
                self.send_goal_future.cancel()

    def ctr_normal(self, plane: LabeledObjectPlane):
        center_pt = (plane.normal_ctr.position.x, plane.normal_ctr.position.y)
        _,_,theta = euler_from_quaternion([plane.normal_ctr.orientation.x, plane.normal_ctr.orientation.y, plane.normal_ctr.orientation.z, plane.normal_ctr.orientation.w])
        return (center_pt, (np.cos(theta), np.sin(theta)))
    
    def pt_left_right(self, plane: LabeledObjectPlane):
        center_pt, normal = self.ctr_normal(plane)
        line_unit = self.perp_vec(normal)
        return ((center_pt[0]-line_unit[0]*plane.size.y, center_pt[1]-line_unit[1]*plane.size.y), (center_pt[0]+line_unit[0]*plane.size.y, center_pt[1]+line_unit[1]*plane.size.y))


    def plane_cb(self, msg: LabeledObjectPlaneArray):
        if not self.started_task:
            return
        # self.get_logger().info('GOT OBJECTS')
        new_dock_banners = []
        new_boat_banners = []
        obj_plane: LabeledObjectPlane
        for obj_plane in msg.objects:
            if(obj_plane.label in self.boat_labels):
                # pt_left, pt_right = self.pt_left_right(obj_plane)
                # new_boat_banners.append((obj_plane.label, (pt_left, pt_right)))
                # self.get_logger().info(f'BOAT: {self.inv_label_mappings[obj_plane.label]} -> {(pt_left, pt_right).__str__()}')
                ctr, normal = self.ctr_normal(obj_plane)
                new_boat_banners.append((obj_plane.label, (ctr, normal)))
                self.get_logger().info(f'BOAT: {self.inv_label_mappings[obj_plane.label]} -> {(ctr, normal).__str__()}')

        merged_obj_indiv: LabeledObjectPlane
        for merged_obj_indiv in msg.coplanar_indiv:
            if (merged_obj_indiv.label in self.dock_labels):
                self.get_logger().info(f'GOT DOCK')
                self.got_dock = True
                # pt_left, pt_right = self.pt_left_right(merged_obj_indiv)
                # new_dock_banners.append((merged_obj_indiv.label, (pt_left, pt_right)))
                ctr, normal = self.ctr_normal(merged_obj_indiv)
                new_dock_banners.append((merged_obj_indiv.label, (ctr, normal)))

        # update global variables
        self.dock_banners = new_dock_banners
        self.boat_banners = new_boat_banners

        self.get_logger().info(f'GOT {len(self.dock_banners)} SLOTS AND {len(self.boat_banners)} BOATS')

        if(not self.got_dock):
            return

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
                        self.state = DockingState.WAITING_DOCK
                        self.cancel_navigation()
                self.get_logger().info(f'DOCK {self.inv_label_mappings[dock_label]} IS TAKEN')
                continue
            # found empty docking slot
            if(self.selected_slot is None or (self.norm(self.midpoint(self.selected_slot[1][0], self.selected_slot[1][1]), self.robot_pos) > self.norm(dock_ctr, self.robot_pos)) + self.update_slot_dist_thres):
                if self.state == DockingState.NAVIGATING_DOCK:
                    self.cancel_navigation()
                # found an empty one closer
                self.selected_slot = (dock_label, (dock_ctr, dock_normal))
                self.picked_slot = True
                # self.pid.reset()
                self.x_pid.reset()
                self.y_pid.reset()
                self.theta_pid.reset()

        if(self.picked_slot):
            self.get_logger().info(f'WILL DOCK INTO {self.inv_label_mappings[self.selected_slot[0]]}')
    
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
    
    def control_loop(self):
        if(not self.picked_slot):
            self.state = DockingState.WAITING_DOCK
            return # maybe stop the robot? or just go forward/steer to the left
        marker_arr = MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)])
        mark_id = 1
        # self.get_logger().info(f'CONTROL LOOP')
        # PID to go to the detected slot (consider its middle and the angle of the whole dock line)
        slot_back_mid = self.selected_slot[1][0]
        slot_dir = self.selected_slot[1][1]
        
        marker_arr.markers.append(VisualizationTools.visualize_line(slot_back_mid, self.perp_vec(slot_dir), mark_id, (0.0, 0.0, 1.0), self.robot_frame_id))
        mark_id = mark_id + 1

        if self.norm(slot_back_mid, self.robot_pos) < self.navigation_dist_thres:
            if self.state == DockingState.NAVIGATING_DOCK:
                self.cancel_navigation()
            self.state = DockingState.DOCKING
            # go to that line and forward (negative error if boat left of line, positive if right)
            offset = self.dot(self.difference(slot_back_mid, self.robot_pos), self.perp_vec(slot_dir))
            approach_angle = self.angle_vec(self.negative(slot_dir), self.robot_dir) # TODO Check sign

            # dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
            # self.pid.update(offset + self.boat_angle_coeff*approach_angle, dt)            
            # yaw_rate = self.pid.get_effort()
            # self.prev_update_time = self.get_clock().now()

            # control_msg = ControlOption()
            # control_msg.priority = 1

            # forward speed decreasing exponentially as we get closer
            dist_diff = self.norm(slot_back_mid, self.robot_pos) - self.dock_length/2.0 # can improve to compute distance from projected center of dock instead, more accurate but not needed since we're gonna be aligned with it at some point anyways
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
            x_output = self.x_pid.get_effort()
            y_output = self.y_pid.get_effort()
            theta_output = self.theta_pid.get_effort()
            x_vel = x_output
            # x_vel = forward_speed
            y_vel = y_output

            x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
            control_msg = ControlOption()
            control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
            control_msg.twist.linear.x = x_vel
            control_msg.twist.linear.y = y_vel
            control_msg.twist.angular.z = theta_output
            self.control_pub.publish(control_msg)

            self.got_dock = False
            self.picked_slot = False
            self.selected_slot = None
        else:
            waypoint = self.sum(slot_back_mid, self.scalar_prod(slot_dir, self.dock_length/2.0))
            if self.state != DockingState.NAVIGATING_DOCK or ((self.sent_waypoint is not None) and (self.norm(waypoint, self.sent_waypoint) > self.adapt_dist)):
                self.get_logger().info('SENDING WAYPOINT')
                # self.get_logger().info(f'passed: {passed_previous}, first passed: {passed_previous}, first buoy pair: {self.first_buoy_pair}, changed pair to: {changed_pair_to}, adapt waypoint: {adapt_waypoint}')
                self.send_waypoint_to_server(waypoint)
                self.state = DockingState.NAVIGATING_DOCK
            elif self.send_goal_future != None and self.lastSelectedGoal != None:
                goal_result = self.send_goal_future.result()
                if self.waypoint_reject or (((goal_result is not None) and (not goal_result.accepted)) or (self.waypoint_sent_future != None and
                    self.waypoint_sent_future.result() != None and 
                    self.waypoint_sent_future.result().status == GoalStatus.STATUS_ABORTED)):
                    self.get_logger().info("Waypoint request aborted by nav server and no new waypoint option found. Resending request...")
                    self.send_waypoint_to_server(self.lastSelectedGoal)
            
        self.marker_pub.publish(marker_arr)

    def execute_callback(self, goal_handle):
        self.start_process("docking starting")
        self.started_task = True
        self.set_pid_setpoints(0, 0, 0)

        while not self.result:

            if self.should_abort():
                self.end_process("aborting docking")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("cancelling docking")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)
        
        self.end_process("docking completed!")
        goal_handle.succeed()
        return Task.Result(success=True)


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