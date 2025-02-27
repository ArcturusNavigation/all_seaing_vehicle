#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from all_seaing_common.action_server_base import ActionServerBase
from all_seaing_controller.pid_controller import PIDController
from all_seaing_driver.central_hub import Buck, Mechanisms
from all_seaing_interfaces.action import Task
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray

import serial
import time

TIMER_PERIOD = 1 / 60
SERVO_HALF_RANGE = 90.0

class DeliveryServer(ActionServerBase):
    def __init__(self):
        super().__init__("delivery_server")

        # --------------- PARAMETERS ---------------#

        Kpid = (
            self.declare_parameter("Kpid", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        port = (
            self.declare_parameter("serial_port", "/dev/ttyACM0")
            .get_parameter_value()
            .string_value
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
        self.water_delivery_time = (
            self.declare_parameter("water_delivery_time", 10.0)
            .get_parameter_value()
            .double_value
        )
        self.object_delivery_time = (
            self.declare_parameter("object_delivery_time", 10.0)
            .get_parameter_value()
            .double_value
        )
        camera_width = (
            self.declare_parameter("camera_width", 640)
            .get_parameter_value()
            .integer_value
        )

        # --------------- SERIAL PORTS ---------------#

        self.ser = serial.Serial(port, 115200, timeout = 1)
        self.buck = Buck(self.ser)
        self.mechanisms = Mechanisms(self.ser)

        # --------------- PID CONTROLLERS ---------------#

        self.aim_pid = PIDController(*Kpid)
        self.aim_pid.set_setpoint(camera_width / 2)   # Want target in center
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
        self.object_sub = self.create_subscription(LabeledBoundingBox2DArray, "bounding_boxes", self.bbox_callback, 10)  # TODO: change topic name
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

        # --------------- MEMBER VARIABLES ---------------#

        self.target_x = 0.0
        self.is_aiming = False


    def timer_callback(self):
        if self.is_aiming:
            self.update_pid()
            effort = self.aim_pid.get_effort()
            servo_output = effort + SERVO_HALF_RANGE
            self.mechanisms.servo2_angle(int(servo_output))
        else:
            self.prev_update_time = self.get_clock().now()
            self.aim_pid.reset()
            self.mechanisms.stop_servo2()

    def bbox_callback(self, msg):
        # TODO: this won't work since you're subscribing to 2d array
        self.target_x = (msg.min_x + msg.max_x) / 2

    def update_pid(self):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.aim_pid.update(self.target_x, dt)
        self.prev_update_time = self.get_clock().now()

    def water_callback(self, goal_handle):
        self.start_process("Water delivery started!")

        self.get_logger().info("Turning on water pump")
        self.is_aiming = True
        self.buck.adj1_en(1)

        time.sleep(self.water_delivery_time)

        self.get_logger().info("Turning off water pump")
        self.is_aiming = False
        self.buck.adj1_en(0)

        self.end_process("Water delivery completed!")
        goal_handle.succeed()
        return Task.Result(success=True)

    def object_callback(self, goal_handle):
        self.start_process("Object delivery started!")

        self.get_logger().info("Turning on ball shooter")
        self.mechanisms.reset_launched()
        self.buck.adj2_voltage(self.shooter_voltage)
        self.buck.adj2_en(1)
        self.mechanisms.servo1_angle(0)
        self.is_aiming = True

        # Aim until a ball is launched or timed out
        start = time.time()
        while self.mechanisms.launched() == 0 and time.time() - start < self.object_delivery_time:
            time.sleep(TIMER_PERIOD)

        self.get_logger().info("Turning off ball shooter")
        self.is_aiming = False
        self.buck.adj2_en(0)
        self.mechanisms.stop_servo1()

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
