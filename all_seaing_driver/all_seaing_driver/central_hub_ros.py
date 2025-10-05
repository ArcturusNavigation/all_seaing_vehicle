#!/usr/bin/env python3



"""
in estpostatus srv, include something about low battery
check this in rover custom controller line 67
"""

import rclpy
from rclpy.node import Node
from all_seaing_driver.ArcturusEE import BMS, main_power, ESTOP, mech_power
from all_seaing_interfaces.srv import CommandAdj, CommandServo, GetEstopStatus, CommandFan, AcknowledgeLowBattery
import serial
import time


class CentralHubROS(Node):
    def __init__(self):
        super().__init__("central_hub_ros")

        port = self.declare_parameter(
            "port", "/dev/ttyACM0").get_parameter_value().string_value
    
        self.low_battery_threshold = self.declare_parameter(
            "low_battery_threshold", 20.0
        ).get_parameter_value().double_value

        ser = serial.Serial(port, 115200, timeout=1)
        self.main_pow = main_power(ser)
        self.bms = BMS(ser, 0x02)
        self.thr_a_bms = BMS(ser, 0x03)
        self.thr_b_bms = BMS(ser, 0x04)
        self.estop = ESTOP(ser)
        self.mech_pow_a = mech_power(ser, 0x05)
        self.mech_pow_b = mech_power(ser, 0x06)

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
        self.cmd_fan_srv = self.create_service(
            CommandFan,
            "command_fan",
            self.cmd_fan_cb,
        )
        
        self.ack_low_battery_srv = self.create_service(
            AcknowledgeLowBattery,
            "acknowledge_low_battery",
            self.ack_low_battery_cb
        )

        self.create_timer(5.0, self.low_battery_check)
        self.battery_ack = False # true if user acknowledges that the battery is low

    def ack_low_battery_cb(self, request, response):
        if request.ack:
            self.battery_ack = True
            self.get_logger().info("Low battery acknowledged.")
        else:
            self.battery_ack = False
            self.get_logger().info("Low battery UN-acknowledged.")
        
        response.success = True
        return response

    def cmd_adj_cb(self, request, response):
        if request.enable:
            if request.port == 1:
                self.mech_pow_a.set_voltage(request.voltage)
                self.mech_pow_a.output(True)
            elif request.port == 2:
                self.mech_pow_b.set_voltage(request.voltage)
                self.mech_pow_b.output(True)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        else:
            if request.port == 1:
                self.mech_pow_a.output(False)
            elif request.port == 2:
                self.mech_pow_b.output(False)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        response.success = True
        return response
    
    
    # TRUE if battery is OKAY, FALSE if battery is LOW
    def battery_check(self):
        check, volt = self.check_battery_cb()
        current_voltage = self.bms.stack_voltage()
        if current_voltage < self.low_battery_threshold and not self.estop.manual():
            if not self.battery_ack:
                self.get_logger().warn(f"Warning: battery is low! {volt}V < {self.low_battery_threshold}V")
                self.get_logger().warn("Please run the command: ros2 service call /acknowledge_low_battery all_seaing_interfaces/srv/AcknowledgeLowBattery \"{acknowledge: true}\" in order to continue")
                return False
        return True

    def cmd_servo_cb(self, request, response):
        # TODO: Implement when we know how many servos we are going to have/how we are going to use them & they are incorporated in the driver file
        # if request.enable:
        #     if request.port == 1:
        #         self.mechanisms.servo1_angle(request.angle)
        #     elif request.port == 2:
        #         self.mechanisms.servo2_angle(request.angle)
        #     else:
        #         self.get_logger().warn("Invalid adj port addressed")
        #         response.success = False
        #         return response
        # else:
        #     if request.port == 1:
        #         self.mechanisms.stop_servo1()
        #     elif request.port == 2:
        #         self.mechanisms.stop_servo2()
        #     else:
        #         self.get_logger().warn("Invalid servo port addressed")
        #         response.success = False
        #         return response
        response.success = False
        return response

    def estop_cb(self, _, response):
        response.success = True
        response.mode = 1 if self.estop.manual() else 0
        response.drive_x2 = -self.estop.drive_x2()
        response.drive_y2 = -self.estop.drive_y2()
        response.drive_x1 = -self.estop.drive_x1()
        response.drive_y1 = -self.estop.drive_y1()
        # TODO: Implement side strafe using drive x2 and y2 (add those to the .srv as well)
        response.is_connected = bool(self.estop.connected())
        response.is_estopped = bool(self.main_pow.estop())

        # Added a field in GetEstopStatus to check if the battery is okay
        response.battery_ok = self.battery_check()
        return response
    
    def cmd_fan_cb(self, request, response):
        if request.enable:
            if request.port == 1:
                self.main_pow.fan1(True)
            elif request.port == 2:
                self.main_pow.fan2(True)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        else:
            if request.port == 1:
                self.main_pow.fan1(False)
            elif request.port == 2:
                self.main_pow.fan2(False)
            else:
                self.get_logger().warn("Invalid adj port addressed")
                response.success = False
                return response
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CentralHubROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
