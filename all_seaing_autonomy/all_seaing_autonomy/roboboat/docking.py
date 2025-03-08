#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import get_package_share_directory

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import LabeledObjectPointCloudArray, LabeledObjectPointCloud, ControlOption
from all_seaing_interfaces.srv import CommandAdj, CommandServo

from all_seaing_autonomy.roboboat.visualization_tools import VisualizationTools

import time
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import math
import yaml
import os
import numpy as np
import random

TIMER_PERIOD = 1 / 30

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

        self.bbox_pcl_sub = self.create_subscription(
            LabeledObjectPointCloudArray, "labeled_object_point_clouds", self.bbox_pcl_cb, qos_profile_sensor_data
        )

        self.marker_pub = self.create_publisher(MarkerArray, 'docking_marker_pub', 10)

        pid_vals = (
            self.declare_parameter("pid_vals", [0.003, 0.0, 0.001]) # TODO: fine-tune values (especially D term)
            .get_parameter_value()
            .double_array_value
        )

        self.declare_parameter("forward_speed", 5.0)
        self.declare_parameter("max_yaw", 1.0)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.declare_parameter("dock_width", 1.0)
        self.declare_parameter("dock_length", 3.0)
        self.dock_width = self.get_parameter("dock_width").get_parameter_value().double_value
        self.dock_length = self.get_parameter("dock_length").get_parameter_value().double_value

        self.declare_parameter("boat_taken_angle_thres", 0.8) # angle of banner-boat to dock segment over pi/2
        self.boat_taken_angle_thres = self.get_parameter("boat_taken_angle_thres").get_parameter_value().double_value

        self.declare_parameter("boat_angle_coeff", 0.8)
        self.boat_angle_coeff = self.get_parameter("boat_angle_coeff").get_parameter_value().double_value

        # RANSAC PARAMS
        self.inlier_threshold = 0.001
        self.num_ransac_iters = 15
        self.inlier_num = 20
        # self.pair_space = 3

        self.pid = PIDController(*pid_vals)
        self.pid.set_effort_min(-self.max_yaw_rate)
        self.pid.set_effort_max(self.max_yaw_rate)
        self.prev_update_time = self.get_clock().now()
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)
        
        # update from subs
        self.labeled_pcl = []
        self.dock_banners = []
        self.dock_banner_line = None
        self.boat_banners = []
        self.selected_slot = None
        self.taken = []

        self.got_dock = False
        self.picked_slot = False

        self.timer_period = 1 / 30.0

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
            label_mappings = yaml.safe_load(f)
        
        self.dock_labels = [label_mappings[name] for name in ["blue_circle", "blue_cross", "blue_triangle", "green_circle", "green_cross", "green_square", "green_triangle", "red_circle", "red_cross", "red_triangle", "red_square"]]
        self.boat_labels = [label_mappings[name] for name in ["black_circle", "black_cross", "black_triangle"]]
    
    def bbox_pcl_cb(self, msg):
        self.get_logger().info('GOT OBJECTS')
        dock_banner_points = []
        new_labeled_pcl = []
        new_dock_banners = []
        new_dock_banner_line = None
        new_boat_banners = []
        marker_arr = MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)])
        mark_id = 1
        for bbox_pcl_msg in msg.objects:
            lidar_point_cloud = pc2.read_points_numpy(bbox_pcl_msg.cloud) # list with shape [num_pts, 3], where second dimension is rgb
            points_2d = [(lidar_point_cloud[i,0], lidar_point_cloud[i,1]) for i in range(lidar_point_cloud.shape[0])] # project points into 2d plane because 3d ransac is hard and I'm def not doing it
            if(len(points_2d) == 0):
                continue
            new_labeled_pcl.append((points_2d, bbox_pcl_msg.label))
            if(bbox_pcl_msg.label in self.boat_labels):
                wall_params, (pt_left, pt_right) = self.find_segment_ransac(points_2d)
                new_boat_banners.append((bbox_pcl_msg.label, wall_params, (pt_left, pt_right)))
                marker_arr.markers.append(VisualizationTools.visualize_segment(pt_left, pt_right, mark_id, (1.0, 0.0, 0.0)))
                mark_id = mark_id + 1
            elif(bbox_pcl_msg.label in self.dock_labels):
                wall_params, (pt_left, pt_right) = self.find_segment_ransac(points_2d)
                new_dock_banners.append((bbox_pcl_msg.label, wall_params, (pt_left, pt_right)))
                dock_banner_points = dock_banner_points + points_2d
                marker_arr.markers.append(VisualizationTools.visualize_segment(pt_left, pt_right, mark_id, (0.0, 0.0, 1.0)))
                mark_id = mark_id + 1
        
        if(not (len(dock_banner_points) == 0)):
            wall_params, (pt_left, pt_right) = self.find_segment_ransac(dock_banner_points)
            new_dock_banner_line = (wall_params, (pt_left, pt_right))
            self.got_dock = True
            marker_arr.markers.append(VisualizationTools.visualize_segment(pt_left, pt_right, mark_id, (0.0, 1.0, 0.0)))
            mark_id = mark_id + 1
            self.dock_banner_line = new_dock_banner_line # don't delete the dock between control updates

        self.marker_pub.publish(marker_arr)

        # update global variables
        self.labeled_pcl = new_labeled_pcl
        self.dock_banners = new_dock_banners
        self.boat_banners = new_boat_banners

        if(not self.got_dock):
            return
        
        # check for taken docks and stuff
        for dock_label, dock_params, (dock_left, dock_right) in self.dock_banners:
            if (dock_label in self.taken):
                continue # just ignore, since we saw that it was taken once it's always taken
            # check if any boat is in that slot
            taken = False
            for boat_label, boat_params, (boat_left, boat_right) in self.boat_banners:
                # check if boat is closer than dock and angle of dock and boat banners is relatively perpendicular to the dock
                mid_boat = self.midpoint(boat_left, boat_right)
                mid_dock = self.midpoint(dock_left, dock_right)
                _, dock_line = self.dock_banner_line
                if (self.norm(mid_boat) < self.norm(mid_dock)) and (abs(self.angle_segments((mid_boat, mid_dock), dock_line)-math.pi/2.0) < self.boat_taken_angle_thres):
                    # taken
                    taken = True
            if taken:
                self.taken.append(dock_label)
                if(self.selected_slot[0] == dock_label):
                    # we're cooked
                    self.selected_slot = None
                    self.picked_slot = False
                    self.pid.reset()
                self.get_logger().info(f'DOCK {dock_label} IS TAKEN')
                continue
            # found empty docking slot
            if(self.selected_slot is not None and (self.norm(self.midpoint(self.selected_slot[1][0], self.selected_slot[1][1])) > self.norm(mid_dock))):
                # found an empty one closer
                self.selected_slot = (dock_label, (dock_left, dock_right))
                self.picked_slot = True
                self.pid.reset()

        self.get_logger().info(f'GOT {len(self.dock_banners)} SLOTS AND {len(self.boat_banners)} BOATS')
        if(self.picked_slot):
            self.get_logger().info(f'WILL DOCK INTO {dock_label}')

    def project_point_line(self, point, line_params):
        x, y = point
        (a, b, c), _ = line_params
        return ((x*b**2-a*b*y-a*c)/(a**2+b**2), (-x*a*b+a**2*y-b*c)/(a**2+b**2))
    
    def distance(self, point, wall_params):
        x, y = point
        (a, b, c), _ = wall_params
        return (a*x+b+y)/math.sqrt(a**2+b**2)

    def midpoint(self, pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        return ((x1+x2)/2.0, (y1+y2)/2.0)

    def difference(self, pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        return (x2-x1, y2-y1)
    
    def norm(self, pt):
        x, y = pt
        return math.sqrt(x**2+y**2)

    def angle_segments(self, p1, p2):
        p1left, p1right = p1
        p2left, p2right = p2
        x1, y1 = self.difference(p1left, p1right)
        x2, y2 = self.difference(p2left, p2right)
        return math.acos(abs(x1*x2+y1*y2)/(self.norm((x1,y1))*self.norm((x2,y2))))
    
    def find_segment_ransac(self, sliced_scan):
        # RANSAC
        wall_params = ((0.0, 0.0, 0.0), 0.0)
        best_num = 0
        count_iters = 0
        avg_error = 0.0
        best_inliers = []
        while best_num < self.inlier_num and count_iters < self.num_ransac_iters:
            i1 = random.randint(0,len(sliced_scan)-1)
            i2 = random.randint(0,len(sliced_scan)-1)
            if(i1==i2):
                continue
            pair_points = (sliced_scan[i1],sliced_scan[i2])
        # for i in range(len(sliced_scan)-self.pair_space): # instead of randomly selecting points, since they are in order of angle, choose pair spaced a certain number of points apart to have a good line
        #     pair_points = (sliced_scan[i], sliced_scan[i+self.pair_space])
        # while best_num < self.inlier_num and count_iters < self.num_ransac_iters:
        #     i1 = random.randint(0,len(sliced_scan)-self.pair_space-1) # combine the above two approaches
            # pair_points = (sliced_scan[i1],sliced_scan[i1+self.pair_space])
            hypothesis_params = self.fit_pair(pair_points)
            # (a,b,c),_ = hypothesis_params
            # self.get_logger().info(f'Wall checked in RANSAC: {a}x + {b}y + {c} = 0')
            num_in, inlier_error, inliers = self.find_inliers(hypothesis_params, sliced_scan)
            if(num_in > best_num):
                wall_params = hypothesis_params
                best_num = num_in
                avg_error = inlier_error
                best_inliers = inliers
            count_iters = count_iters + 1
        # self.get_logger().info(f'Wall found with {best_num} inliers in {count_iters} iterations and average inlier error {avg_error}')
        x_left = y_left = None
        x_right = y_right = None
        for x,y in best_inliers:
            x_proj, y_proj = self.project_point_line((x,y), wall_params)
            if (x_left is None) or (x_proj < x_left) or (y_proj < y_left):
                x_left, y_left = x_proj, y_proj
            if (x_right is None) or (x_proj > x_right) or (y_proj > y_right):
                x_right, y_right = x_proj, y_proj

        return (wall_params, ((x_left, y_left), (x_right, y_right)))
            
    def find_inliers(self, wall_params, points):
        (a, b, c), _ = wall_params
        count = 0
        in_error_avg = 0.0
        inliers = []
        for x,y in points:
            if ((a*x+b*y+c)**2)/(a**2+b**2) < self.inlier_threshold:
                count = count + 1
                in_error_avg = in_error_avg + ((a*x+b*y+c)**2)/(a**2+b**2)
                inliers.append((x,y))
        if(count != 0):
            in_error_avg = in_error_avg/count
        return count, in_error_avg, inliers
    
    def fit_points(self, point_set):
        # linear regression, analytical solution
        x_list, y_list = zip(*point_set)
        X = np.array([x_list]).T
        X_aug = np.hstack([X,np.full((len(x_list),1),1.0)])
        Y = np.array(y_list).T
        try:
            analytic_sol = np.linalg.inv(X_aug.T@X_aug)@(X_aug.T@Y)
        except np.linalg.LinAlgError as error:
            analytic_sol = np.linalg.inv(X_aug.T@X_aug+len(x_list)*0.01*np.eye(2))@(X_aug.T@Y)
        slope, offset = analytic_sol[0], analytic_sol[1]
        distance = abs(offset)/math.sqrt(slope**2+1)
        return ((slope, offset), distance)
    
    def fit_pair(self, point_pair):
        (x1,y1), (x2,y2) = point_pair
        # (y-y1)(x2-x1) = (y2-y1)*(x-x1) -> y(x2-x1)+x(y1-y2)+x1(y2-y1)+y1(x1-x2) = 0 -> y(x2-x1)+x(y1-y2)+x1y2-x2y1
        return (y1-y2, x2-x1, x1*y2-x2*y1), abs(x1*y2-x2*y1)/math.sqrt((y1-y2)**2+(x2-x1)**2)
    
    def control_loop(self):
        if(not self.picked_slot):
            return # maybe stop the robot? or just go forward/steer to the left
        
        # PID to go to the detected slot (consider its middle and the angle of the whole dock line)
        slot_back_mid = self.midpoint(self.selected_slot[1][0], self.selected_slot[1][1])
        (a, b, c), _ = self.dock_banner_line
        perp_dock_line_params = (-b, a, 
                                 b*slot_back_mid[0] - a*slot_back_mid[1])

        # go to that line and forward (positive error if boat left of line, negative if right)
        # TODO: check the signs
        offset = -math.copysign(-perp_dock_line_params[0]*perp_dock_line_params[2])*abs(perp_dock_line_params[2]/math.sqrt(perp_dock_line_params[0]**2+perp_dock_line_params[1]**2))
        if(abs(perp_dock_line_params[1]) < 0.0001):
            approach_angle = np.pi/2.0
        else:
            approach_angle = math.atan(-perp_dock_line_params[0]/perp_dock_line_params[1])
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.pid.update(offset + self.boat_angle_coeff*approach_angle, dt)            
        yaw_rate = self.pid.get_effort()
        self.prev_update_time = self.get_clock().now()

        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        self.control_pub.publish(control_msg)

        self.got_dock = False
        self.dock_banner_line = None
        self.picked_slot = False
        self.selected_slot = None

    def execute_callback(self, goal_handle):
        self.start_process("docking starting")

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