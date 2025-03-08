#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from all_seaing_driver.central_hub import Buck, ESTOP, Mechanisms
from all_seaing_interfaces.srv import CommandAdj, CommandServo, GetEstopStatus
import serial
import time

CMD_PERIOD = 1 / 2
TIMER_PERIOD = 1/ 30

class CentralHubROS(Node):
    def __init__(self):
        super().__init__("central_hub_ros")

        port = self.declare_parameter(
            "port", "/dev/ttyACM0").get_parameter_value().string_value
        ser = serial.Serial(port, 115200, timeout=1)
        self.buck = Buck(ser)
        self.estop = ESTOP(ser)
        self.mechanisms = Mechanisms(ser)

        self.cmd_adj_srv = self.create_service(
            CommandAdj,
            "command_adj",
            self.cmd_adj_cb
        )
        self.cmd_servo_srv = self.create_service(
            CommandServo,
            "command_servo",
            self.cmd_servo_cb,
        )
        self.get_estop_srv = self.create_service(
            GetEstopStatus,
            "get_estop_status",
            self.estop_cb,
        )
        self.prev = self.get_clock().now()

    def cmd_adj_cb(self, request, response):
        while (self.get_clock().now() - self.prev).nanoseconds / 1e9 < CMD_PERIOD:
            time.sleep(TIMER_PERIOD)

        if request.enable:
            if request.port == 1:
                self.buck.adj1_voltage(request.voltage)
                self.buck.adj1_en(1)
            elif request.port == 2:
                self.buck.adj2_voltage(request.voltage)
                self.buck.adj2_en(1)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        else:
            if request.port == 1:
                self.buck.adj1_en(0)
            elif request.port == 2:
                self.buck.adj2_en(0)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        response.success = True
        self.prev = self.get_clock().now()
        return response

    def cmd_servo_cb(self, request, response):
        while (self.get_clock().now() - self.prev).nanoseconds / 1e9 < CMD_PERIOD:
            time.sleep(TIMER_PERIOD)

        if request.enable:
            if request.port == 1:
                self.mechanisms.servo1_angle(request.angle)
            elif request.port == 2:
                self.mechanisms.servo2_angle(request.angle)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        else:
            if request.port == 1:
                self.mechanisms.stop_servo1()
            elif request.port == 2:
                self.mechanisms.stop_servo2()
            else:
                self.get_logger().warn("Invalid servo port addressed")
                response.success = False
                return response
        response.success = True
        self.prev = self.get_clock().now()
        return response

    def estop_cb(self, _, response):
        while (self.get_clock().now() - self.prev).nanoseconds / 1e9 < CMD_PERIOD:
            time.sleep(TIMER_PERIOD)

        response.success = True
        response.mode = self.estop.mode()
        response.drive_x = self.estop.drive_x()
        response.drive_y = self.estop.drive_y()
        response.is_connected = bool(self.estop.connected())
        response.is_estopped = bool(self.estop.estop())
        self.prev = self.get_clock().now()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CentralHubROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
