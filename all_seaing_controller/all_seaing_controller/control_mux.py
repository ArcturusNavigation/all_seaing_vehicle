#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ControlOption
from geometry_msgs.msg import Twist
import time

from all_seaing_common.run_logger import RunLogger
from all_seaing_common.status_reporter import StatusReporter

PRIORITY_NAMES = {0: "teleop", 1: "autonomy"}  # lowest value wins

TIMER_PERIOD = 1 / 2

class ControlMux(Node):
    def __init__(self):
        super().__init__("control_mux")

        self.control_sub = self.create_subscription(
            ControlOption, "control_options", self.control_callback, 10
        )
        self.control_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.received_messages = {}
        self.run_log = RunLogger(self)
        self.status = StatusReporter(self)
        self.winner = None  # priority currently forwarded to cmd_vel
        self._last_cmd_log = 0.0

    # Control callback should be called more frequently than timer_callback
    def control_callback(self, msg):
        self.received_messages[msg.priority] = msg.twist

        # Publish the highest priority message, i.e. the lowest priority value
        winner = min(self.received_messages.keys())
        twist = self.received_messages[winner]
        self.control_pub.publish(twist)
        cmd = [round(twist.linear.x, 3), round(twist.linear.y, 3), round(twist.angular.z, 3)]
        name = PRIORITY_NAMES.get(winner, f"priority {winner}")
        if winner != self.winner:  # source switch (e.g. teleop took over)
            self.run_log.info("mux_source", source=name, priority=winner,
                              previous=PRIORITY_NAMES.get(self.winner, self.winner), cmd=cmd)
            self.status.count("source_switches")
            self.winner = winner
        self.status.gauge("source", name)
        if True:  # every cmd_vel
            self.run_log.debug("cmd_vel", source=name, cmd=cmd, n_sources=len(self.received_messages))

    def timer_callback(self):
        if self.received_messages:
            self.status.count("stale_clears")
        self.received_messages = {}


def main():
    rclpy.init()
    control_mux = ControlMux()
    rclpy.spin(control_mux)
    control_mux.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
