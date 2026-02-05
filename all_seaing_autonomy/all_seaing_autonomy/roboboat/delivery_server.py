#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray
from all_seaing_interfaces.srv import CommandAdj, CommandServo

import time

TIMER_PERIOD = 1 / 10
SERVO_HALF_RANGE = 90.0

class DeliveryServer(ActionServerBase):
    serial_instance = None

    def __init__(self):
        super().__init__("delivery_server")

        # --------------- PARAMETERS ---------------#

        Kpid = (
            self.declare_parameter("Kpid", [0.18, 0.0, 0.0])
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
        self.object_sub = self.create_subscription(LabeledBoundingBox2DArray, "shape_boxes", self.bbox_callback, 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

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


    def timer_callback(self):
        if not self.is_sim:
            if self.is_aiming:
                self.update_pid()
                effort = self.aim_pid.get_effort()
                # TODO this doesn't make sense as a PID controller (zero steady state error is literally impossible)
                # convert to applying a certain velocity at each point & integrating (keeping track of angle at each step) to update the sent angle
                # then when error is 0 we have 0 angle diff thus same angle, stabilizing the turret
                servo_output = SERVO_HALF_RANGE + effort
                req = CommandServo.Request()
                req.enable = True
                req.angle = int(servo_output)
                req.port = 2
                self.command_servo_cli.call_async(req)
            else:
                self.prev_update_time = self.get_clock().now()
                self.aim_pid.reset()
                req = CommandServo.Request()
                req.enable = False
                req.port = 2
                self.command_servo_cli.call_async(req)

    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    def update_pid(self):
        # TODO if not find bbox with certain label, search right and left (sweep)
        largest_bbox_area = 0
        # TODO add a parametrizable list of bbox labels to only consider those, to not start shooting towards the dock or something else
        # TODO set a flag when the server is called to only consider the water or ball labels based on the request
        for bbox in self.bboxes:
            area = (bbox.max_x - bbox.min_x) * (bbox.max_y - bbox.min_y)
            if area > largest_bbox_area:
                largest_bbox_area = area
                self.target_x = (bbox.min_x + bbox.max_x) / 2
        self.aim_pid.set_setpoint(self.camera_width / 2)   # Want target in center
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.aim_pid.update(self.target_x, dt)
        self.prev_update_time = self.get_clock().now()

    def water_callback(self, goal_handle):
        self.start_process("Water delivery started!")

        if not self.is_sim:
            self.get_logger().info("Turning on water pump")
            req = CommandAdj.Request()
            req.enable = True
            req.port = 1
            req.voltage = 12.0
            self.command_adj_cli.call_async(req)
            self.is_aiming = True

        time.sleep(self.water_delivery_time)
        
        if not self.is_sim:
            self.get_logger().info("Turning off water pump")
            req = CommandAdj.Request()
            req.enable = False
            req.port = 1
            self.command_adj_cli.call_async(req)
            self.is_aiming = False

        self.end_process("Water delivery completed!")
        goal_handle.succeed()
        
        return Task.Result(success=True)

    def object_callback(self, goal_handle):
        self.start_process("Object delivery started!")

        if not self.is_sim:
            self.get_logger().info("Turning on ball shooter")

            # Turn on ball shooter motors
            req = CommandAdj.Request()
            req.enable = True
            req.port = 2
            req.voltage = 5.0
            self.command_adj_cli.call_async(req)

            # Turn on ball shooter feeding servo
            req = CommandServo.Request()
            req.enable = True
            req.angle = 0
            req.port = 1
            self.command_servo_cli.call_async(req)
            self.is_aiming = True

        # Aim until a ball is launched or timed out
        start = time.time()
        while time.time() - start < self.object_delivery_time:
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
            req.enable = False
            req.port = 1
            self.command_servo_cli.call_async(req)
            self.is_aiming = False

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
