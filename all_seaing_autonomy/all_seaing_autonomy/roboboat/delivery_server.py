#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray
from all_seaing_interfaces.srv import CommandAdj, CommandServo
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool

import time
import yaml
import os

TIMER_PERIOD = 1 / 10
SERVO_HALF_RANGE = 180.0
SERVO_MAX = 240.0
SERVO_MIN = 120.0
SWEEP_MIN = 120.0
SWEEP_MAX = 240.0
SWEEP_OMEGA = 45.0
SERVO_STATION = 180
SERVO_INITIAL = 180

class DeliveryServer(ActionServerBase):
    def __init__(self):
        super().__init__("delivery_server")

        # --------------- PARAMETERS ---------------#

        Kpid = (
            self.declare_parameter("Kpid", [0.2, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.aim_threshold = (
            self.declare_parameter("aim_threshold", 5)
            .get_parameter_value()
            .integer_value
        )
        self.shooter_voltage = (
            self.declare_parameter("shooter_voltage", 5)
            .get_parameter_value()
            .integer_value
        )
        self.water_voltage = (
            self.declare_parameter("water_voltage", 12)
            .get_parameter_value()
            .integer_value
        )
        self.water_delivery_time = (
            self.declare_parameter("water_delivery_time", 5.0)
            .get_parameter_value()
            .double_value
        )
        self.object_delivery_time = (
            self.declare_parameter("object_delivery_time", 5.0)
            .get_parameter_value()
            .double_value
        )
        self.aiming_timeout = (
            self.declare_parameter("aiming_timeout", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.camera_width = (
            self.declare_parameter("camera_width", 640)
            .get_parameter_value()
            .integer_value
        )

        self.declare_parameter("is_sim", False)
        self.is_sim = self.get_parameter("is_sim").get_parameter_value().bool_value

        # --------------- PID CONTROLLERS ---------------#

        self.aim_pid = PIDController(*Kpid)
        self.aim_pid.set_integral_min(-SERVO_HALF_RANGE)
        self.aim_pid.set_integral_max(SERVO_HALF_RANGE)
        self.aim_pid.set_effort_min(-SERVO_HALF_RANGE)
        self.aim_pid.set_effort_max(SERVO_HALF_RANGE)
        self.prev_update_time = self.get_clock().now()

        # --------------- SUBS, PUBS, TIMERS, AND SERVERS ---------------#

        self.water_delivery_server = ActionServer(
            self,
            Task,
            "water_delivery",
            execute_callback=self.water_callback,
            cancel_callback=self.default_cancel_callback,
        )
        self.object_delivery_server = ActionServer(
            self,
            Task,
            "object_delivery",
            execute_callback=self.object_callback,
            cancel_callback=self.default_cancel_callback,
        )
        self.object_sub = self.create_subscription(LabeledBoundingBox2DArray, "shape_boxes", self.bbox_callback, 1)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        self.switch_sub = self.create_subscription(Bool, "switch_status", self.switch_cb, 10)

        if not self.is_sim:
            self.command_adj_cli = self.create_client(CommandAdj, "command_adj")
            self.command_servo_cli = self.create_client(CommandServo, "command_servo")
            while not self.command_adj_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("CommandAdj service not available, waiting again...")
                time.sleep(TIMER_PERIOD)
            while not self.command_servo_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("CommandServo service not available, waiting again...")
                time.sleep(TIMER_PERIOD)

        # --------------- MEMBER VARIABLES ---------------#

        self.target_x = self.camera_width / 2
        self.is_aiming = False
        self.bboxes = []
        self.servo_angle = SERVO_HALF_RANGE
        self.sweep_sign = 1
        self.no_aiming_start = -1

        bringup_prefix = get_package_share_directory("all_seaing_bringup")

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

        self.target_labels = []

        self.switch_status = True

        if not self.is_sim:
            # turn on turret servo
            req = CommandAdj.Request()
            req.enable = True
            req.port = 1
            req.voltage = 7.4
            self.command_adj_cli.call_async(req)

            req = CommandServo.Request()
            req.enable = True
            req.port = 1
            req.angle = SERVO_STATION
            self.command_servo_cli.call_async(req)

            # Turn off ball shooter feeding servo
            req = CommandServo.Request()
            req.enable = True
            req.angle = 90 # do nothing
            req.port = 2
            self.command_servo_cli.call_async(req)

    def timer_callback(self):
        if not self.is_sim:
            if self.is_aiming:
                self.update_pid()
                servo_output = self.servo_angle
                req = CommandServo.Request()
                req.enable = True
                req.angle = int(servo_output)
                req.port = 1
                self.command_servo_cli.call_async(req)

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    def switch_cb(self, msg):
        self.switch_status = msg.data

    def update_pid(self):
        largest_bbox_area = 0
        for bbox in self.bboxes:
            # self.get_logger().info(f'BBOX LABEL: {bbox.label}')
            if bbox.label not in self.target_labels:
                continue
            # self.get_logger().info(f'FOUND BBOX')
            area = (bbox.max_x - bbox.min_x) * (bbox.max_y - bbox.min_y)
            if area > largest_bbox_area:
                largest_bbox_area = area
                self.target_x = (bbox.min_x + bbox.max_x) / 2
                self.sweep_sign = 1 if self.target_x < self.camera_width/2 else -1 # will start sweeping towards the direction we last saw it
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.prev_update_time = self.get_clock().now()
        if largest_bbox_area == 0:
            if self.no_aiming_start == -1:
                self.no_aiming_start = time.time()
            elif time.time() - self.no_aiming_start > self.aiming_timeout:
                # if not find bbox with certain label, search right and left (sweep)
                if self.servo_angle <= SWEEP_MIN:
                    self.sweep_sign = 1
                elif self.servo_angle >= SWEEP_MAX:
                    self.sweep_sign = -1
                self.servo_angle += self.sweep_sign*SWEEP_OMEGA*dt
                self.servo_angle = min(max(self.servo_angle, SWEEP_MIN), SWEEP_MAX)
                # self.get_logger().info(f"SWEEPING, ANGLE={self.servo_angle}")
                return
        else:
            self.no_aiming_start = -1
            self.aim_pid.set_setpoint(self.camera_width / 2)   # Want target in center
            self.aim_pid.update(self.target_x, dt)
            effort = self.aim_pid.get_effort()
            self.servo_angle += effort*dt # effort is considered to be omega basically
            self.servo_angle = min(max(self.servo_angle, SERVO_MIN), SERVO_MAX)
            # self.get_logger().info(f"AIMING, ANGLE={self.servo_angle}")

    def water_callback(self, goal_handle):
        self.start_process("Water delivery started!")

        self.target_labels = self.water_labels
        self.prev_update_time = self.get_clock().now()
        self.servo_angle = SERVO_INITIAL
        self.sweep_sign = 1

        if not self.is_sim:
            # turn on turret servo
            req = CommandServo.Request()
            req.enable = True
            req.port = 1
            req.angle = SERVO_INITIAL
            self.command_servo_cli.call_async(req)

            self.is_aiming = True

            self.get_logger().info("Turning on water pump")
            req = CommandAdj.Request()
            req.enable = True
            req.port = 3
            req.voltage = 12.0
            self.command_adj_cli.call_async(req)

        time.sleep(self.water_delivery_time)
        
        if not self.is_sim:
            self.get_logger().info("Turning off water pump")
            req = CommandAdj.Request()
            req.enable = False
            req.port = 3
            self.command_adj_cli.call_async(req)

            self.is_aiming = False

            # turn off turret servo
            req = CommandServo.Request()
            req.enable = True
            req.port = 1
            req.angle = SERVO_STATION
            self.command_servo_cli.call_async(req)

        self.target_labels = []

        self.end_process("Water delivery completed!")
        goal_handle.succeed()
        
        return Task.Result(success=True)

    def object_callback(self, goal_handle):
        self.start_process("Object delivery started!")

        self.target_labels = self.ball_labels
        self.prev_update_time = self.get_clock().now()
        self.servo_angle = SERVO_HALF_RANGE
        self.sweep_sign = 1

        if not self.is_sim:
            # turn on turret servo
            req = CommandServo.Request()
            req.enable = True
            req.port = 1
            req.angle = SERVO_INITIAL
            self.command_servo_cli.call_async(req)

            self.is_aiming = True

            self.get_logger().info("Turning on ball shooter")

            # Turn on ball shooter motors
            req = CommandAdj.Request()
            req.enable = True
            req.port = 2
            req.voltage = 12.0
            self.command_adj_cli.call_async(req)

            # Turn on ball shooter feeding servo
            req = CommandServo.Request()
            req.enable = True
            req.angle = 0 # push, then 180 is to go back, and 90 is do nothing
            req.port = 2
            self.command_servo_cli.call_async(req)
            
        # Aim until a ball is launched or timed out
        prev_status = False
        curr_status = False
        start = time.time()
        while time.time() - start < self.object_delivery_time and not (prev_status == True and curr_status == False):
            prev_status = curr_status
            curr_status = self.switch_status
            time.sleep(TIMER_PERIOD)
        
        if not self.is_sim:
            self.get_logger().info("Turning off ball shooter")

            # Turn off ball shooter motors
            req = CommandAdj.Request()
            req.enable = False
            req.port = 2
            self.command_adj_cli.call_async(req)

            # Turn off ball shooter feeding servo
            req = CommandServo.Request()
            req.enable = True
            req.angle = 90 # do nothing
            req.port = 2
            self.command_servo_cli.call_async(req)
            self.is_aiming = False

            # turn off turret servo
            req = CommandServo.Request()
            req.enable = True
            req.port = 1
            req.angle = SERVO_STATION
            self.command_servo_cli.call_async(req)

        self.target_labels = []

        self.end_process("Object delivery completed!")
        goal_handle.succeed()
        return Task.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
