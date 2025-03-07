#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import Task
from all_seaing_controller.pid_controller import PIDController
from ament_index_python.packages import get_package_share_directory
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption
from all_seaing_common.action_server_base import ActionServerBase
from sensor_msgs.msg import CameraInfo

import os
import yaml
import time

class FollowBuoyPID(ActionServerBase):
    def __init__(self):
        super().__init__("follow_path_pid_server")

        self._action_server = ActionServer(
            self,
            Task,
            "follow_buoy_pid",
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
        
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        # update from subs
        self.height = None
        self.width = None
        self.bboxes = []

        self.timer_period = 1 / 30.0

        self.green_labels = set()
        self.red_labels = set()
        self.yellow_labels = set()

        self.result = False
        self.seen_first_buoy = False

        self.scale_right = 1.0

        
        self.declare_parameter(
            "color_label_mappings_file", 
            os.path.join(
                bringup_prefix, "config", "perception", "color_label_mappings.yaml"
            ),
        )

        self.red_green_ratio = None

        color_label_mappings_file = self.get_parameter(
            "color_label_mappings_file"
        ).value
        with open(color_label_mappings_file, "r") as f:
            label_mappings = yaml.safe_load(f)
        # hardcoded from reading YAML
        self.green_labels.add(label_mappings["green"])
        self.red_labels.add(label_mappings["red"])
        if self.is_sim:
            self.yellow_labels.add(label_mappings["black"])
        else:
            self.yellow_labels.add(label_mappings["yellow"])

        


    def intrinsics_callback(self, msg):
        self.height = msg.height
        self.width = msg.width

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    def control_loop(self):
        if self.width is None or len(self.bboxes) == 0:
            return
        

        red_center_x = None
        red_area = 0

        green_center_x = None
        green_area = 0

        # yellow_center_x = None
        yellow_area = 0
        yellow_left = None
        yellow_right = None

        # handle balancing box sizes later ?
        # ex. same size but offset should be y shift
        # but diff size should be a rotation and probably 
        # a bit of a y shift too?

        for box in self.bboxes:
            area = (box.max_x - box.min_x) * (box.max_y - box.min_y)
            midpt = (box.max_x + box.min_x) / 2.0
            if box.label in self.green_labels and area > green_area:
                green_area = area
                green_center_x = midpt
            elif box.label in self.red_labels and area > red_area:
                red_area = area
                red_center_x = midpt
            elif box.label in self.yellow_labels and area > yellow_area:
                yellow_area = area
                yellow_left = box.min_x
                yellow_right = box.max_x
                
        # if we only see one of red / green, rotate
        # if we see neither, log + kill
        # if red_center_x is None or green_center_x is None:
        #     if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 1.0:
        #         self.get_logger().info("no more buoys killing")
        #         self.result = True
        #     return

        yaw0 = False
        if red_center_x is None and green_center_x is None:
            yaw0 = True
            if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 1.0:
                self.get_logger().info("no more buoys killing")
                self.result = True
            return
        else:
            self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        if red_center_x is not None and green_center_x is not None:
            self.red_green_ratio = red_area / green_area
        # keep going in the same direction if the ratio is too far off.
        # if self.red_green_ratio is not None and (self.red_green_ratio > 0.5 or self.red_green_ratio < 0.5):
        #     yaw0 = True
        #     return

        # can update, not impt rn
        left_x = red_center_x
        right_x = green_center_x
        img_ctr = self.width / 2.0

        if left_x is None: 
            if right_x < img_ctr:
                left_x = right_x - (self.width * 0.75)
            else:
                left_x = 0
        if right_x is None: 
            if left_x >= img_ctr:
                right_x = left_x + (self.width * 0.75)
            else:
                right_x = self.width - 1

        gate_ctr = (left_x + right_x) / 2.0
        left_ctr_thresh = img_ctr * 0.45
        right_ctr_thresh = img_ctr * 0.55
        offset = None

        # yellow buoy exists
        if yellow_left is not None:
            # if yellow_left <= right_ctr_thresh and yellow_right >= right_ctr_thresh:

            yellow_ctr = (yellow_left + yellow_right) / 2.0
            if yellow_left <= img_ctr and yellow_right >= img_ctr and left_x <= yellow_ctr <= right_x:
                # yellow buoy is in the middle (on both sides of camera)
                left_diff = img_ctr - yellow_left
                right_diff = yellow_right - img_ctr
                if left_diff < right_diff:
                    # yellow buoy is on the right side
                    # want to turn left
                    # goal is to get yellow_left to align with right_ctr_thresh 
                    # should overshoot a bit ?
                    # bc this would stop running once its past the center. 
                    self.get_logger().info("yellow buoy on the right side. turning left. ")
                    offset = yellow_left - right_ctr_thresh

                else:
                    self.get_logger().info("yellow buoy is on the left side. turning right.")
                    offset = yellow_right - left_ctr_thresh
                    # yellow buoy is on the left side
                    # want to turn right
                    # goal is to get yellow_right to align with left_ctr_thresh
        if offset is None:
            offset = gate_ctr - self.width / 2.0


        #     if yellow_left > img_ctr and yellow_right > img_ctr:
        #         # both vals are on the right of center

        #     elif yellow_left <= img_ctr and yellow_right <= img_ctr:
        #         # both vals are on the left of center
        # if yellow_center_x is not None:
        #     if yellow_area > green_area*0.8 or yellow_area < green_area*2:
        #         if yellow_center_x > left_x and yellow_center_x < right_x:
        #             if gate_ctr > yellow_center_x:
        #                 ctr = (right_x + yellow_center_x) / 2.0
        #                 offset = ctr - self.width / 2.0
        #             elif gate_ctr <= yellow_center_x:
        #                 ctr = (left_x + yellow_center_x) / 2.0
        #                 offset = ctr - self.width / 2.0
                # else:
                #     # self.width / 2.0 is img ctr
                #     offset = gate_ctr - self.width / 2.0



        # self.width / 2.0 is img ctr
        # offset = gate_ctr - self.width / 2.0

        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.pid.update(offset, dt)            
        yaw_rate = self.pid.get_effort()
        if yaw_rate < 0.0: # 3/6: if turning rihgt, make turn larger
            yaw_rate = max(yaw_rate * self.scale_right, -self.max_yaw_rate)
        if yaw0:
            yaw_rate = 0.0
        self.prev_update_time = self.get_clock().now()

        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        self.control_pub.publish(control_msg)

    def execute_callback(self, goal_handle):
        self.start_process("follow buoy pid starting")
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        while not self.result:

            if self.should_abort():
                self.end_process("aborting follow buoy pid")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("cancelling follow buoy pid")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)
        
        self.end_process("follow buoy pid completed!")
        goal_handle.succeed()
        return Task.Result(success=True)



def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPID()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

    