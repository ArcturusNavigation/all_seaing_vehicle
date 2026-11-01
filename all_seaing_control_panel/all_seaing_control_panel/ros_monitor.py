"""
The control panel's ROS node (runs on the laptop, in a background thread).

Only light topics are subscribed to over the network, all with best-effort QoS, so a lossy
Wi-Fi link drops messages instead of stalling publishers on the Jetson (reliable delivery
over Wi-Fi is what made RViz on the laptop slow the boat down). Heavy topics (images,
point clouds) are measured on the Jetson by topic_monitor.py and arrive here as
/diagnostics.

Keeps:
- the latest DiagnosticStatus per name (+ receive time) from /diagnostics
- a TF buffer from /tf (best effort) and /tf_static (transient local, as published)
- a short history of odometry (/odometry/gps) and GPS fixes for the heading check
"""

import math
import threading
import time
from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosMonitor(Node):
    def __init__(self, topics):
        super().__init__("arcturus_control_panel")
        self._lock = threading.Lock()
        self.diagnostics = {}  # name -> dict
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self._static_tfs = []  # re-inserted after a time jump clears the buffer
        self._last_tf_stamp = 0.0
        self.odom = deque(maxlen=600)  # (wall time, yaw, vx_body, vy_body)
        self.fixes = deque(maxlen=200)  # (wall time, lat, lon)

        self.create_subscription(DiagnosticArray, topics.get("diagnostics", "/diagnostics"), self._diag_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data)
        static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._tf_static_cb, static_qos)
        self.create_subscription(Odometry, topics.get("odometry", "/odometry/gps"), self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(NavSatFix, topics.get("gps", "/mavros/global_position/raw/fix"), self._fix_cb,
                                 qos_profile_sensor_data)

    def _diag_cb(self, msg):
        now = time.time()
        with self._lock:
            for s in msg.status:
                level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
                self.diagnostics[s.name] = {
                    "name": s.name, "level": LEVEL_NAMES.get(level, str(level)), "message": s.message,
                    "hardware_id": s.hardware_id, "values": {kv.key: kv.value for kv in s.values}, "received": now}

    def _tf_cb(self, msg):
        for t in msg.transforms:
            stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            if stamp < self._last_tf_stamp - 1.0:
                # time jumped back (looping rosbag replay, sim restart): drop the dynamic history,
                # as RViz does, instead of rejecting all new data as too old
                self.tf_buffer.clear()
                with self._lock:
                    static = list(self._static_tfs)
                for s in static:
                    self.tf_buffer.set_transform_static(s, "control_panel")
            self._last_tf_stamp = stamp
            self.tf_buffer.set_transform(t, "control_panel")

    def _tf_static_cb(self, msg):
        with self._lock:
            self._static_tfs.extend(msg.transforms)
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, "control_panel")

    def _odom_cb(self, msg):
        with self._lock:
            self.odom.append((time.time(), yaw_from_quaternion(msg.pose.pose.orientation),
                              msg.twist.twist.linear.x, msg.twist.twist.linear.y))

    def _fix_cb(self, msg):
        if msg.status.status >= 0:
            with self._lock:
                self.fixes.append((time.time(), msg.latitude, msg.longitude))

    # ------------------------------------------------------------------ queries (thread-safe)

    def diagnostics_snapshot(self, max_age=10.0):
        now = time.time()
        with self._lock:
            out = []
            for d in self.diagnostics.values():
                d = dict(d)
                d["age"] = now - d["received"]
                if d["age"] > max_age:
                    d["level"], d["message"] = "STALE", f"no status for {d['age']:.0f} s"
                out.append(d)
        return sorted(out, key=lambda d: d["name"])

    def can_transform(self, target, source):
        try:
            return self.tf_buffer.can_transform(target, source, rclpy.time.Time())
        except Exception:
            return False

    def tf_frames(self):
        try:
            return self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return ""

    def heading_check(self, window=15.0, min_speed=0.5):
        """
        Compare the odometry heading with the GPS course over the last `window` seconds.
        Returns a dict with status ok/warn/fail/unknown. Needs the boat to move in a straight-ish
        line faster than min_speed [m/s].
        """
        now = time.time()
        with self._lock:
            fixes = [f for f in self.fixes if now - f[0] <= window]
            odom = [o for o in self.odom if now - o[0] <= window]
        if len(fixes) < 2 or not odom:
            return {"status": "unknown", "message": "no recent GPS fixes / odometry (is the boat running?)"}
        (t0, lat0, lon0), (t1, lat1, lon1) = fixes[0], fixes[-1]
        north = (lat1 - lat0) * 111_320.0
        east = (lon1 - lon0) * 111_320.0 * math.cos(math.radians(lat0))
        dist = math.hypot(north, east)
        speed = dist / max(t1 - t0, 1e-6)
        if speed < min_speed:
            return {"status": "unknown",
                    "message": f"drive forward in a straight line (> {min_speed} m/s); GPS speed now {speed:.2f} m/s"}
        course_enu = math.atan2(north, east)  # ENU yaw convention (0 = east, CCW), as in odometry
        yaw = math.atan2(sum(math.sin(o[1]) for o in odom), sum(math.cos(o[1]) for o in odom))
        diff = math.degrees((yaw - course_enu + math.pi) % (2 * math.pi) - math.pi)
        vx = sum(o[2] for o in odom) / len(odom)
        details = {"gps_course_deg": round(math.degrees(course_enu), 1), "odom_yaw_deg": round(math.degrees(yaw), 1),
                   "difference_deg": round(diff, 1), "gps_speed": round(speed, 2), "odom_vx_body": round(vx, 2)}
        if abs(diff) > 45 or vx < 0:
            return {"status": "fail", "details": details,
                    "message": f"heading differs from GPS course by {diff:.0f} deg"
                               + (" and body vx < 0 (axes flipped?)" if vx < 0 else "")}
        if abs(diff) > 20:
            return {"status": "warn", "details": details, "message": f"heading differs from GPS course by {diff:.0f} deg"}
        return {"status": "ok", "details": details, "message": f"heading matches GPS course within {abs(diff):.0f} deg"}


class RosMonitorThread:
    """Runs RosMonitor in its own executor thread."""

    def __init__(self, topics):
        rclpy.init()
        self.node = RosMonitor(topics)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    def shutdown(self):
        self.node.destroy_node()
        rclpy.try_shutdown()
