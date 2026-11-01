"""
Live per-component status for Python ROS 2 nodes (issue #480). Python counterpart of
all_seaing_common/status_reporter.hpp (same keys, levels and semantics).

A node records what its pipeline is doing as named counters (e.g. "frames_in",
"rejected/sync_timeout"), gauges (latest value) and timings (e.g. "inference_ms"). A
background thread, independent of the node's executor so a slow callback doesn't delay
it, publishes them every `status_period` seconds on /diagnostics as one
diagnostic_msgs/DiagnosticStatus named "<node>[/<component>]", viewable in Foxglove's
Diagnostics panel or with `ros2 topic echo /diagnostics`. For each period:
    counters: "<name>" = total since start, "<name>/rate" = per second over the last period
    gauges:   "<name>" = latest value
    timings:  "<name>/mean", "<name>/max", "<name>/count" over the last period
    "period_s", plus a level (OK / WARN / ERROR / STALE) and a message
The level is STALE when the heartbeat counter (default "updates") didn't change during a
period, WARN/ERROR when set with set_level(), else OK.

Recording calls only take a lock briefly and never do I/O. This is independent of
RunLogger (#501): use RunLogger for per-event detail, StatusReporter for live aggregates.

Usage:
    from all_seaing_common.status_reporter import StatusReporter
    self.status = StatusReporter(self)              # or StatusReporter(self, "front")
    self.status.count("updates")                    # once per pipeline iteration (heartbeat)
    self.status.count("rejected/low_confidence", n)
    self.status.gauge("boxes_in_frame", n)
    with self.status.timer("inference_ms"):         # or self.status.timing("inference_ms", ms)
        ...

ROS parameters (declared on the node): status_enabled (bool, True),
status_period (double, 1.0 s), status_heartbeat (str, "updates").
"""

import threading
import time
from contextlib import contextmanager

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _fmt(v):
    return f"{v:.4g}" if isinstance(v, float) else str(v)


class StatusReporter:

    def __init__(self, node, component=""):
        self.enabled = node.declare_parameter("status_enabled", True).value
        self._period = node.declare_parameter("status_period", 1.0).value
        self._heartbeat = node.declare_parameter("status_heartbeat", "updates").value
        self._lock = threading.Lock()
        self._counters, self._counters_prev = {}, {}
        self._gauges = {}
        self._timings = {}  # name -> [sum, max, n]
        self._level, self._message = DiagnosticStatus.OK, "OK"
        if not self.enabled:
            return

        self._hardware_id = node.get_fully_qualified_name()
        self._name = self._hardware_id.lstrip("/") + (f"/{component}" if component else "")
        self._clock = node.get_clock()
        self._pub = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._last_publish = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="status_reporter", daemon=True)
        self._thread.start()

    def count(self, name, n=1):
        if self.enabled:
            with self._lock:
                self._counters[name] = self._counters.get(name, 0) + n

    def gauge(self, name, value):
        if self.enabled:
            with self._lock:
                self._gauges[name] = _fmt(value)

    def timing(self, name, ms):
        if self.enabled:
            with self._lock:
                t = self._timings.setdefault(name, [0.0, 0.0, 0])
                t[0] += ms
                t[1] = max(t[1], ms)
                t[2] += 1

    @contextmanager
    def timer(self, name):
        """Records the elapsed time [ms] of the with-block under `name`."""
        start = time.perf_counter()
        try:
            yield
        finally:
            self.timing(name, (time.perf_counter() - start) * 1e3)

    def set_level(self, level, message):
        """Level/message (DiagnosticStatus.OK/WARN/ERROR as bytes or int) shown until changed."""
        if self.enabled:
            with self._lock:
                self._level, self._message = level, message

    def close(self):
        if self.enabled:
            self._stop.set()

    def _build(self):
        with self._lock:
            now = time.monotonic()
            dt, self._last_publish = now - self._last_publish, now
            values = [KeyValue(key="period_s", value=_fmt(dt))]
            for name, total in sorted(self._counters.items()):
                prev = self._counters_prev.get(name, 0)
                values.append(KeyValue(key=name, value=str(total)))
                values.append(KeyValue(key=f"{name}/rate", value=_fmt((total - prev) / dt if dt > 0 else 0.0)))
            for name, value in sorted(self._gauges.items()):
                values.append(KeyValue(key=name, value=value))
            for name, (s, mx, n) in sorted(self._timings.items()):
                values += [KeyValue(key=f"{name}/mean", value=_fmt(s / n if n else 0.0)),
                           KeyValue(key=f"{name}/max", value=_fmt(mx)),
                           KeyValue(key=f"{name}/count", value=str(n))]
            self._timings = {}  # timings are per period

            hb, hb_prev = self._counters.get(self._heartbeat, 0), self._counters_prev.get(self._heartbeat, 0)
            if hb == hb_prev:
                level, message = DiagnosticStatus.STALE, f"no '{self._heartbeat}' in the last {dt:.3g} s"
            else:
                level, message = self._level, self._message
            self._counters_prev = dict(self._counters)

        status = DiagnosticStatus(name=self._name, hardware_id=self._hardware_id, message=message, values=values)
        status.level = level if isinstance(level, bytes) else bytes([level])
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = self._clock.now().to_msg()
        return msg

    def _run(self):
        while not self._stop.wait(self._period):
            try:
                self._pub.publish(self._build())
            except Exception:  # context shut down mid-publish etc.: never take the node down
                pass
