#!/usr/bin/env python3

import rclpy
from rclpy.node import Node # imports Node class from ros2 packages
from rclpy.action import ActionServer
from all_seaing_driver.driver_library import Buck, Mechanisms
from all_seaing_interfaces.action import Delivery
import serial
import time

# from all_seaing_interfaces.msg import ControlOption, Heartbeat
# from geometry_msgs.msg import Twist

#TODO: Write action client for perception OR figure out who to report result back to
#TODO: How to localize delivery boats and flag as delivered

class ObjectDelivery(Node):
    def __init__(self):
        super().__init__("object_delivery_node")

        self.declare_parameter("serial_port", "/dev/ttyACM0")

        self.ser = serial.Serial(self.get_parameter("serial_port").value, 115200, timeout = 1)
        self.buck = Buck(self.ser)
        self.mechanisms = Mechanisms(self.ser)
        self.target_speed = 0
        # self.launch_begin = False

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

    def water_callback(self, goal_handle):
        target_angle = goal_handle.request.target
        self.get_logger().info(f'Goal Received: Aiming water pump with target angle {target_angle}.')
        # wait_time = goal_handle.request.target
        feedback_msg = Delivery.Feedback()

        self.buck.adj2_en(1)
        feedback_msg.status = 'Turret ON'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(5)

        self.mechanisms.servo1_angle(target_angle)
        feedback_msg.status = 'Aim IN PROGRESS'
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(10)

        self.mechanisms.stop_servo1()

        self.buck.adj1_en(1)
        feedback_msg.status = 'Pump ON'
        goal_handle.publish_feedback(feedback_msg)

        # time.sleep(wait_time) #TODO: uncomment later
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
        self.buck.adj2_en(1)
        feedback_msg.status = 'Ball launcher ON'
        goal_handle.publish_feedback(feedback_msg)

        #TODO: Change standard servo angle
        self.mechanisms.servo1_angle(target_angle)
        feedback_msg.status = 'Aim IN PROGRESS'
        goal_handle.publish_feedback(feedback_msg)

        self.mechanisms.servo2_angle(self.target_speed)

        while self.mechanisms.launched() == 0:
            feedback_msg.status = 'Launch IN PROGRESS'
            goal_handle.publish_feedback(feedback_msg)

        self.mechanisms.reset_launched()

        self.mechanisms.stop_servo1()
        self.mechanisms.stop_servo2()

        self.buck.adj2_en(0)
        feedback_msg.status = 'Ball launcher OFF'
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
