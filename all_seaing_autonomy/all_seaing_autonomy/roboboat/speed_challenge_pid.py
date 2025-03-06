#!/usr/bin/env python3
from ast import Num
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from all_seaing_controller.pid_controller import PIDController


from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, LabeledBoundingBox2D, ControlOption
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import CameraInfo, Imu
from all_seaing_common.action_server_base import ActionServerBase
from tf_transformations import euler_from_quaternion

import os
import yaml
import math
import time
from collections import deque

TIMER_PERIOD = 1 / 60

class SpeedChange(ActionServerBase):
    def __init__(self):
        super().__init__("speed_challenge_server")

        self._action_server = ActionServer(
            self,
            Task,
            "speed_challenge",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_cb, 10
        )

        self.bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray, 
            "bounding_boxes", 
            self.bbox_callback, 
            10
        )

        self.imu_sub = self.create_subscription(
            Imu, "/mavros/imu/data/", self.imu_cb, 10
        )

        self.control_pub = self.create_publisher(
            ControlOption, 
            "control_options", 
            10
        )


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




        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value
        self.declare_parameter("turn_offset", 1.0)
        self.turn_offset = self.get_parameter("turn_offset").get_parameter_value().double_value
        
        # NOTE: in qualifying round we assume we enter from the correct direction.


        # unit vector in the direction of the blue buoy
        # ex: (0, -1) for south (-y), (0,1) for north (+y)

        self.image_size = (400,400)  
        self.bboxes = []      

        self.blue_labels = set()
        self.red_labels = set()
        self.green_labels = set()
        self.loops = []
        self.blue_x = 0
        self.current_loop_index = 0
        self.prev_loop_index = 0

        self.starting_yaw = 0


        # TODO: change the param to be the same between is_sim and not
        # too sleepy, dont want to break things.
        # CODE IS COPIED FROM FOLLOW_BUOY_PATH,SUBJECT TO CHANGES
        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter(
            "color_label_mappings_file",
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )

        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)
        self.red_labels.add(label_mappings["red"])
        self.green_labels.add(label_mappings["green"])
        self.blue_labels.add(label_mappings["blue"])

    def reset_challenge(self):
        '''
        Readies the server for the upcoming speed challenge.
        '''
        self.current_loop_index = 0
        self.loops
        pass

    def camera_info_cb(self, msg):
        '''
        Gets camera image info from all_seaing_perception.
        '''
        self.image_size = (msg.width, msg.height)
    
    def imu_cb(self, msg):
        self.imu_ang_vel = msg.angular_velocity
        self.imu_orientation = msg.orientation
        self.imu_lin_acc = msg.linear_acceleration

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    def execute_callback(self, goal_handle):
        self.start_process("Speed challenge task started!")

        self.reset_challenge()
        self.get_logger().info("Speed challenge setup completed.")


        # FTB
        # Trace a long path and turn when blue buoy detected
        # trace a somewhat long path and run FTB when gates are detected

        while rclpy.ok():
            # Check if the action client requested cancel
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancel requested. Aborting task initialization.")
                goal_handle.canceled()
                return Task.Result(success=False)


            self.run_loop()
            if (self.current_loop_index == 5):
                self.get_logger().info("Speed challenge task successfully ended.")
                return Task.Result(success=True)
                
            time.sleep(TIMER_PERIOD)

        # If we exit the `while rclpy.ok()` loop somehow
        self.get_logger().info("ROS shutdown detected or loop ended unexpectedly.")
        goal_handle.abort()
        return Task.Result(success=False)
    
    def run_loop(self):
        '''
        Control which PID loop is being ran.
        '''
        self.prev_loop_index = self.current_loop_index
        if self.current_loop_index == 0: #follow buoy
            self.follow_buoy_pid()
        elif self.current_loop_index == 1: #go straight
            self.go_straight_pid()
            self.blue_trigger()
        elif self.current_loop_index == 2: #circle around
            self.blue_buoy_pid()

            yaw_diff = self.get_yaw()-self.starting_yaw
            if yaw_diff > 180:
                yaw_diff -= 180
            elif yaw_diff < -180:
                yaw_diff += 180
            if (abs(yaw_diff) > 170):
                self.current_loop_index += 1
            # use imu rotation data to exit (a bit over 180 degrees?)
            pass
        elif self.current_loop_index == 3: #go straight back
            self.go_straight_pid()
            self.redgreen_trigger()
            pass
        elif self.current_loop_index == 4: #
            self.follow_buoy_pid(False)


        if self.current_loop_index != self.prev_loop_index:
                self.starting_yaw = self.get_yaw()

    def blue_trigger(self):
        for box in self.bboxes:
            if box.label in self.blue_labels:
                self.current_loop_index += 1
                midpt = (box.max_x + box.min_x) / 2.0
                self.blue_x = midpt
                return

    def redgreen_trigger(self):
        for box in self.bboxes:
            if box.label in self.red_labels:
                self.current_loop_index += 1
                return
            if box.label in self.green_labels:
                self.current_loop_index += 1
                return
            
    def go_straight_pid(self):
        yaw = self.get_yaw()

        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        offset = yaw-self.starting_yaw
        self.pid.update(offset, dt)            
        yaw_rate = self.pid.get_effort()
        self.prev_update_time = self.get_clock().now()

        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        self.control_pub.publish(control_msg)
    
    def blue_buoy_pid(self):
        blue_center_x = self.image_size[0]/2
        blue_area = 0

        for box in self.bboxes:
            area = (box.max_x - box.min_x) * (box.max_y - box.min_y)
            midpt = (box.max_x + box.min_x) / 2.0
            if box.label in self.blue_labels and area > blue_area:
                blue_area = area
                blue_center_x = midpt

        offset = blue_center_x - self.blue_x

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
        

    def follow_buoy_pid(self, greenRight=True):
        red_center_x = None
        red_area = 0

        green_center_x = None
        green_area = 0

        for box in self.bboxes:
            area = (box.max_x - box.min_x) * (box.max_y - box.min_y)
            midpt = (box.max_x + box.min_x) / 2.0
            if box.label in self.green_labels and area > green_area:
                green_area = area
                green_center_x = midpt
            elif box.label in self.red_labels and area > red_area:
                red_area = area
                red_center_x = midpt
        
        if red_center_x is None and green_center_x is None:
            if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 1.0:
                self.get_logger().info("no more buoys killing")
                self.current_loop_index += 1
            return
        else:
            if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 6.0:
                self.get_logger().info("no more buoys killing")
                self.current_loop_index += 1
                return
            self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        left_x = red_center_x
        right_x = green_center_x

        if left_x is None: left_x = 0
        if right_x is None: right_x = self.image_size[0] - 1

        gate_ctr = (left_x + right_x) / 2.0

        # self.image_size[0] / 2.0 is img ctr
        offset = gate_ctr - self.image_size[0] / 2.0

        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.pid.update(offset, dt)            
        yaw_rate = self.pid.get_effort()
        self.prev_update_time = self.get_clock().now()

        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        if not greenRight:
            control_msg.twist.angular.z *= -1
        self.control_pub.publish(control_msg)

    def get_euler(quat):
        (row, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return row, pitch, yaw
    
    def get_yaw(self):
        return self.get_euler(self.imu_orientation)[2]
    



def main(args=None):
    rclpy.init(args=args)
    node = SpeedChange()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
