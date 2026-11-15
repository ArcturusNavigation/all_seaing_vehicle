#!/usr/bin/env python3
"""
Topic monitor (issue #480): measures the rate and timestamp age of selected topics on the
machine it runs on (the Jetson), and publishes them on /diagnostics, one status per topic,
named "topic_monitor/<topic>". This lets the control panel / Foxglove see the health of
heavy sensor topics (images, point clouds) without subscribing to them over Wi-Fi.

Cheap by design: subscriptions are raw (messages are not deserialized). The header stamp is
read directly from the serialized CDR bytes, for types that start with a std_msgs/Header.
Subscriptions use best-effort QoS, which matches both reliable and best-effort publishers.
Topics that don't exist yet are picked up as soon as they appear.

Per-topic status values (StatusReporter conventions, see all_seaing_common):
    messages, messages/rate      count and rate [Hz] over the last period
    stamp_age_ms                 now - header.stamp of the latest message (if the type has a header)
    expected_min_rate            from the config
Level: ERROR when no message was received in the last period, WARN when the rate is
below expected_min_rate or |stamp_age_ms| > max_stamp_age_ms, else OK.

Parameters:
    config_file (string): YAML with a list `topics:` of {name, min_rate (Hz, optional),
        header (bool, optional, default: guessed from the type)} and an optional
        `max_stamp_age_ms` (default 1000).
        Default: all_seaing_bringup/config/topic_monitor.yaml

Usage: ros2 run all_seaing_utility topic_monitor.py [--ros-args -p config_file:=<yaml>]
"""

import os
import struct
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from all_seaing_common.status_reporter import StatusReporter

# Types whose first field is a std_msgs/Header, so the stamp follows the 4-byte CDR encapsulation header
HEADER_FIRST_PREFIXES = ("sensor_msgs/msg/", "nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped",
                         "geometry_msgs/msg/TwistStamped", "geometry_msgs/msg/PointStamped",
                         "all_seaing_interfaces/msg/LabeledBoundingBox2DArray", "diagnostic_msgs/msg/DiagnosticArray")


class MonitoredTopic:
    def __init__(self, node, cfg, max_stamp_age_ms):
        self.name = cfg["name"]
        self.min_rate = float(cfg.get("min_rate", 0.0))
        self.header = cfg.get("header")  # None: decide from the type once known
        self.max_stamp_age_ms = max_stamp_age_ms
        self.status = StatusReporter(node, self.name.lstrip("/"))
        self.status.gauge("expected_min_rate", self.min_rate)
        self.status.set_level(DiagnosticStatus.ERROR, "topic not published")
        self.sub = None
        self.count = 0
        self.lock = threading.Lock()
        self.last_stamp_ns = None

    def subscribe(self, node, type_name):
        if self.header is None:
            self.header = type_name.startswith(HEADER_FIRST_PREFIXES)
        self.sub = node.create_subscription(get_message(type_name), self.name, self.callback,
                                            qos_profile_sensor_data, raw=True)
        self.status.gauge("type", type_name)

    def callback(self, data):
        self.status.count("messages")
        with self.lock:
            self.count += 1
            if self.header and len(data) >= 12 and data[1] == 0x01:  # CDR little endian
                sec, nanosec = struct.unpack_from("<iI", data, 4)
                self.last_stamp_ns = sec * 1_000_000_000 + nanosec

    def update(self, now_ns, period):
        """Set the level/message for the coming report from the last period's message count."""
        with self.lock:
            count, self.count = self.count, 0
            stamp = self.last_stamp_ns
        if self.sub is None:
            self.status.set_level(DiagnosticStatus.ERROR, "topic not published")
            return
        rate = count / period
        age_ms = (now_ns - stamp) / 1e6 if stamp is not None else None
        if age_ms is not None:
            self.status.gauge("stamp_age_ms", age_ms)
        if count == 0:
            self.status.set_level(DiagnosticStatus.ERROR, "no messages")
        elif rate < self.min_rate:
            # the measured rate itself is in `messages/rate` (same window as the other values)
            self.status.set_level(DiagnosticStatus.WARN, f"rate below {self.min_rate:g} Hz")
        elif age_ms is not None and abs(age_ms) > self.max_stamp_age_ms:
            self.status.set_level(DiagnosticStatus.WARN, f"stamp age {age_ms:.0f} ms (clock/time sync?)")
        else:
            self.status.set_level(DiagnosticStatus.OK, "OK")


class TopicMonitor(Node):
    def __init__(self):
        super().__init__("topic_monitor")
        # A topic's status is STALE when it received no messages in a period
        self.declare_parameter("status_heartbeat", "messages")
        default_cfg = os.path.join(get_package_share_directory("all_seaing_bringup"), "config", "topic_monitor.yaml")
        cfg_file = self.declare_parameter("config_file", default_cfg).value
        with open(cfg_file) as f:
            cfg = yaml.safe_load(f)
        max_age = float(cfg.get("max_stamp_age_ms", 1000.0))
        self.topics = [MonitoredTopic(self, t, max_age) for t in cfg["topics"]]
        self.period = self.get_parameter("status_period").value
        # Subscribe to topics as they appear, and evaluate each period
        self.create_timer(2.0, self.discover)
        self.create_timer(self.period, self.evaluate)
        self.discover()

    def discover(self):
        types = dict(self.get_topic_names_and_types())
        for t in self.topics:
            if t.sub is None and t.name in types and types[t.name]:
                t.subscribe(self, types[t.name][0])

    def evaluate(self):
        now_ns = self.get_clock().now().nanoseconds
        for t in self.topics:
            t.update(now_ns, self.period)


def main():
    rclpy.init()
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
