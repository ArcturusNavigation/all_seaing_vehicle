#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, ActionClient
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

class MovingDelivery(ActionServerBase):
    def __init__(self):
        super().__init__("moving_delivery_server")

        self._action_server = ActionServer(
            self,
            Task,
            "moving_delivery_server",
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
        self.water_client = ActionClient(self, Task, "water_delivery_server")
        self.ball_client = ActionClient(self, Task, "ball_delivery_server")

        self.declare_parameter("forward_speed", 5.0)
        self.declare_parameter("max_yaw", 1.0)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.pid = PIDController(*pid_vals)
        self.pid.set_effort_min(-self.max_yaw_rate)
        self.pid.set_effort_max(self.max_yaw_rate)
        self.prev_update_time = self.get_clock().now()

        bringup_prefix = get_package_share_directory("all_seaing_bringup")
        self.declare_parameter("is_sim", False)

        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        # update from subs
        self.height = None
        self.width = None
        self.bboxes = None

        self.timer_period = 1 / 30.0

        self.cross_labels = set()
        self.triangle_labels = set()

        # change this to shapes
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
        self.cross_labels.add(label_mappings["black_cross"])
        self.triangle_labels.add(label_mappings["black_triangle"])


    def intrinsics_callback(self, msg):
        self.height = msg.height
        self.width = msg.width

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    def call_delivery_server(self, delivery_type="water"):
        if delivery_type == "water":
            client = self.water_client
        elif delivery_type == "ball":
            client = self.ball_client

        goal = Task.Goal()
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().info("didnt work for some reason")
            return False
        

        if future.result().result.success:
            self.get_logger().info(f"{delivery_type} delivery succeeded!")
            return True

    
    def send_control_msg(self, x, y, angular):
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = x
        control_msg.twist.linear.y = y
        control_msg.twist.angular.z = angular
        self.control_pub.publish(control_msg)
        return

    def control_loop(self):
        if self.width is None or self.bboxes is None:
            return

        threshold_width = 10 # need to tune

        cross_center_x = None
        triangle_center_x = None

        cross_width = 0
        triangle_width = 0

        for box in self.bboxes:
            midpt = (box.max_x + box.min_x) / 2.0
            width = box.max_x - box.min_x
            if box.label in self.cross_labels and width > cross_width:
                cross_center_x = midpt
                cross_width = width
            elif box.label in self.triangle_labels and width > triangle_width:
                triangle_center_x = midpt
                triangle_width = width

        if cross_center_x is None and triangle_center_x is None:
            # turn slowly
            control_msg = ControlOption()
            control_msg.priority = 1
            control_msg.twist.linear.x = 0
            control_msg.twist.linear.y = 0
            control_msg.twist.angular.z = float(self.max_yaw_rate * 0.1)
            self.control_pub.publish(control_msg)
            return

        if triangle_center_x is None:
            if cross_width > threshold_width:
                if self.call_delivery_server("ball"):
                    self.result = True
                    return

            else:
                self.send_control_msg(float(self.forward_speed * 0.2), 0, 0)
        elif cross_center_x is None:
            if triangle_width >= threshold_width:
                self.send_control_msg(0, 0, 0)
                # Call on water delivery
                if self.call_delivery_server("water"):
                    self.result = True
                    return
            else:
                self.send_control_msg(float(self.forward_speed * 0.2), 0, 0)

        else:
            if triangle_width >= cross_width:
                if triangle_width >= threshold_width:
                    self.send_control_msg(0, 0, 0)
                    # Call on water delivery
                    if self.call_delivery_server("water"):
                        self.result = True
                        return
                else:
                    self.send_control_msg(float(self.forward_speed * 0.2), 0, 0)

            else:
                if cross_width > threshold_width:
                    self.send_control_msg(0, 0, 0)
                    # Call on ball delivery
                    if self.call_delivery_server("ball"):
                        self.result = True
                        return
                    
                else:
                    self.send_control_msg(float(self.forward_speed * 0.2), 0, 0)


        # publish control at the end
        control_msg = ControlOption()
        control_msg.priority = 1
        control_msg.twist.linear.x = float(self.forward_speed)
        control_msg.twist.linear.y = 0.0
        control_msg.twist.angular.z = float(yaw_rate)
        self.control_pub.publish(control_msg)

    def execute_callback(self, goal_handle):
        self.start_process("starting moving to delivery vessels")

        while not self.result:

            if self.should_abort():
                self.end_process("aborting moving to delivery")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("cancelling moving to delivery")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)

        self.end_process("moving to delivery completed!")
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
