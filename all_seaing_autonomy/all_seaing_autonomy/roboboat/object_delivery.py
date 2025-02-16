#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from all_seaing_driver.central_hub import Buck, Mechanisms
from all_seaing_interfaces.action import Delivery
from all_seaing_controller.pid_controller import PIDController
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray

import serial
import time

class ObjectDelivery(Node):
    def __init__(self):
        super().__init__("object_delivery_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")

        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.buck = Buck(self.ser)
        self.mechanisms = Mechanisms(self.ser)
        self.target_speed = 0

        self.center_x = float("inf")
        self.center_y = float("inf")

        self.object_sub = self.create_subscription(LabeledBoundingBox2DArray, "bounding_boxes", self.compute_center, 10)

        # PID controls for servo 2
        Kpid = (
            self.declare_parameter("Kpid", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )

        # self.max_vel = (
        #     self.declare_parameter("max_vel", [4.0, 2.0, 1.0])
        #     .get_parameter_value()
        #     .double_array_value
        # )

        self.servo1_pid = PIDController(*Kpid)

        self.servo1_pid.set_setpoint(0)
        # self.servo1_pid.set_effort_min(-self.max_vel[2])
        # self.servo1_pid.set_effort_max(self.max_vel[2]) #TODO: change parameter values

        self.prev_update_time = self.get_clock().now()
        self.threshold = 0.5 #TODO: Change

        # Action servers
        self.water_delivery_server = ActionServer(
            self,
            Delivery,
            'water_delivery',
            self.water_callback)

        self.object_delivery_server = ActionServer(
            self,
            Delivery,
            'object_delivery',
            self.object_callback
        )

    def compute_center(self, msg):
        self.center_x = (msg.min_x + msg.max_x) / 2

    def update_pid(self):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.servo1_pid.update(self.center_x, dt)
        self.prev_update_time = self.get_clock().now()

    def control_loop(self, feedback_msg):
        self.update_pid()
        servo1_output = self.servo1_pid.get_effort()
        self.mechanisms.servo1_angle(servo1_output)

        feedback_msg.status = "Aim IN PROGRESS" #TODO: feedback report

    def water_callback(self, goal_handle):
        target_angle = goal_handle.request.target
        self.get_logger().info(f'Goal Received: Aiming water pump with target angle {target_angle}.')
        feedback_msg = Delivery.Feedback()

        self.prev_update_time = self.get_clock().now()
        while self.servo1_pid.is_done(0, self.threshold):
            self.control_loop(feedback_msg)
            goal_handle.publish_feedback(feedback_msg)
            #TODO: may need a time.sleep()
        self.servo1_pid.reset()

        self.mechanisms.stop_servo1()

        self.buck.adj1_en(1)
        feedback_msg.status = 'Pump ON'
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(5)

        self.buck.adj1_en(0)
        feedback_msg.status = 'Pump OFF'
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Delivery.Result()
        result.delivered = True
        return result

    def object_callback(self, goal_handle):
        target_angle = goal_handle.request.target
        self.get_logger().info(f'Goal Received: Aiming servo with target angle of {target_angle}.')

        feedback_msg = Delivery.Feedback()

        # TODO: Fix feedback?
        self.buck.adj2_voltage(12)
        self.buck.adj2_en(1)
        feedback_msg.status = "Ball launcher ON"
        goal_handle.publish_feedback(feedback_msg)

        self.prev_update_time = self.get_clock().now()
        while self.servo1_pid.is_done(0, self.threshold):
            self.control_loop(feedback_msg)
            goal_handle.publish_feedback(feedback_msg)
            #TODO: may need a time.sleep()
        self.servo1_pid.reset()

        self.mechanisms.servo2_angle(self.target_speed)

        self.mechanisms.reset_launched()
        while self.mechanisms.launched() == 0:
            feedback_msg.status = "Launch IN PROGRESS"
            goal_handle.publish_feedback(feedback_msg)

        self.mechanisms.reset_launched()

        self.mechanisms.stop_servo1()
        self.mechanisms.stop_servo2()

        self.buck.adj2_en(0)
        feedback_msg.status = "Ball launcher OFF"
        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Delivery.Result()
        result.delivered = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDelivery()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
