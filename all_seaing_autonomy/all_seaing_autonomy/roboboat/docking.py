#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import Point, PointStamped

from ament_index_python.packages import get_package_share_directory

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption
from all_seaing_interfaces.srv import CommandAdj, CommandServo

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

        self.bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray, 
            "bounding_boxes", 
            self.bbox_callback, 
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

        self.point_cloud_sub = self.create_subscription(
            PointCloud2, "/point_cloud/filtered", self.point_cloud_cb, qos_profile_sensor_data
        )

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        pid_vals = (
            self.declare_parameter("pid_vals", [0.003, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )

        self.declare_parameter("forward_speed", 5.0)
        self.declare_parameter("max_yaw", 1.0)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.pid = PIDController(*pid_vals)
        self.pid.set_effort_min(-self.max_yaw_rate)
        self.pid.set_effort_max(self.max_yaw_rate)
        self.prev_update_time = self.get_clock().now()
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)
        
        # update from subs
        self.height = None
        self.width = None
        self.bboxes = []
        self.points_2d = None

        self.timer_period = 1 / 30.0

        self.state = None
        self.result = False

        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        self.declare_parameter(
            "buoy_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "buoy_label_mappings.yaml"
            ),
        )

        buoy_label_mappings_file = self.get_parameter(
            "buoy_label_mappings_file"
        ).value
        with open(buoy_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)
    
    def intrinsics_callback(self, msg):
        self.height = msg.height
        self.width = msg.width

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes
    
    def point_cloud_cb(self, msg):
        lidar_point_cloud = pc2.read_points_numpy(msg) # list with shape [num_pts, 3], where second dimension is rgb
        self.points_2d = lidar_point_cloud[:,:2] # project points into 2d plane because 3d ransac is hard and I'm def not doing it

    def find_wall_ransac(self, sliced_scan):
        # RANSAC
        wall_params = ((0.0, 0.0, 0.0), 0.0)
        best_num = 0
        count_iters = 0
        avg_error = 0.0
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
            num_in, inlier_error = self.num_inliers(hypothesis_params, sliced_scan)
            if(num_in > best_num):
                wall_params = hypothesis_params
                best_num = num_in
                avg_error = inlier_error
            count_iters = count_iters + 1
        # self.get_logger().info(f'Wall found with {best_num} inliers in {count_iters} iterations and average inlier error {avg_error}')
        return wall_params
            
    def num_inliers(self, wall_params, points):
        (a, b, c), _ = wall_params
        count = 0
        in_error_avg = 0.0
        for x,y in points:
            if ((a*x+b*y+c)**2)/(a**2+b**2) < self.inlier_threshold:
                count = count + 1
                in_error_avg = in_error_avg + ((a*x+b*y+c)**2)/(a**2+b**2)
        if(count != 0):
            in_error_avg = in_error_avg/count
        return count, in_error_avg
    
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

        offset = 0

        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.pid.update(offset, dt)            
        yaw_rate = self.pid.get_effort()
        self.prev_update_time = self.get_clock().now()

        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        self.control_pub.publish(control_msg)


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