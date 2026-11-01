"""
Per-run structured text logger for ROS 2 nodes (issue #501).

Every node gets one JSON-lines file per run, recording its status, decisions and key data
(e.g. task state changes, chosen gates, rejected detections and why, processing times).
The log is much lighter than a rosbag but enough to reconstruct what happened in a run.
`all_seaing_utility/run_log_analysis.py` reads these files.

Design guarantees:
- Never blocks the node: `log()` only appends a dict to a bounded in-memory queue. A
  background thread serializes entries and does all file I/O.
- Survives I/O failures: write and persist errors are caught, counted and retried. They are
  never raised into the node.
- Bounded memory: the queue keeps at most `queue_size` entries and drops the oldest. The
  RAM buffer file keeps at most `max_buffer_bytes`. All drops are counted and reported
  in the log itself (`event: "run_logger_dropped"`).
- Fast writes, durable storage: entries are written to a RAM-backed buffer file
  (`$ARCTURUS_LOG_BUFFER_DIR`, default `/dev/shm/arcturus_logs`). Every `persist_period`
  seconds, and on shutdown, the buffered bytes are appended to the persistent file
  (`$ARCTURUS_LOG_DIR`, default `~/arcturus_logs`) and the buffer is cleared.
- Recovery: if a node dies without shutting down (SIGKILL, crash), its buffer file
  stays in /dev/shm. Any later logger start, or
  `python3 -m all_seaing_common.run_logger --recover` (run by kill_ros_nodes.sh), appends
  it to the persistent file. Only a power loss loses data, and at most `persist_period`
  seconds of it.

Run grouping: all nodes started during one run write into the same `<run_id>` folder.
The run id comes from `$ARCTURUS_RUN_ID` if set. Otherwise the first logger creates
`<buffer_dir>/current_run` (containing a timestamp id) and later loggers reuse it.
`--end-run` (run by kill_ros_nodes.sh) removes it, so the next launch starts a new run.

File layout:
    <log_dir>/<run_id>/<node>.jsonl            persistent (one per node per run)
    <buffer_dir>/<run_id>/<node>.<pid>.jsonl   RAM buffer (unpersisted tail only)

Entry format (one JSON object per line):
    {"t": <ROS time [s]>, "wall": <wall time [s]>, "node": "<node>", "level": "INFO",
     "event": "<event name>", <field>: <value>, ...}

Usage in a node:
    from all_seaing_common.run_logger import RunLogger
    self.run_log = RunLogger(self)                  # parameters: see RunLogger.__init__
    self.run_log.info("gate_selected", left=[x1, y1], right=[x2, y2], dist=d)
    self.run_log.warn("detection_rejected", camera="front", label="red_buoy", reason="hsv_mismatch")
Field values must be JSON-serializable after a `str()` fallback. numpy scalars and arrays
are converted. Pass copies of anything that's mutated later, because serialization
happens on the background thread.
"""

import atexit
import collections
import datetime
import json
import os
import sys
import threading
import time

ENV_LOG_DIR = "ARCTURUS_LOG_DIR"
ENV_BUFFER_DIR = "ARCTURUS_LOG_BUFFER_DIR"
ENV_RUN_ID = "ARCTURUS_RUN_ID"
DEFAULT_LOG_DIR = os.path.expanduser("~/arcturus_logs")
DEFAULT_BUFFER_DIR = "/dev/shm/arcturus_logs"
CURRENT_RUN_FILE = "current_run"


def log_dir():
    """Persistent log root directory."""
    return os.path.expanduser(os.environ.get(ENV_LOG_DIR, DEFAULT_LOG_DIR))


def buffer_dir():
    """RAM-backed buffer root directory."""
    return os.path.expanduser(os.environ.get(ENV_BUFFER_DIR, DEFAULT_BUFFER_DIR))


def current_run_id():
    """
    Run id shared by all nodes of a run: $ARCTURUS_RUN_ID, else the id in
    <buffer_dir>/current_run, which the first caller creates atomically with a timestamp.
    """
    env_id = os.environ.get(ENV_RUN_ID)
    if env_id:
        return env_id
    marker = os.path.join(buffer_dir(), CURRENT_RUN_FILE)
    os.makedirs(buffer_dir(), exist_ok=True)
    new_id = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    try:
        fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        with os.fdopen(fd, "w") as f:
            f.write(new_id)
        return new_id
    except FileExistsError:
        with open(marker) as f:
            return f.read().strip() or new_id


def end_run():
    """Remove the current-run marker so the next launch starts a new run."""
    try:
        os.remove(os.path.join(buffer_dir(), CURRENT_RUN_FILE))
    except FileNotFoundError:
        pass


def _append_file(src_path, dst_path):
    """Append the whole content of src to dst. Raises OSError on failure."""
    os.makedirs(os.path.dirname(dst_path), exist_ok=True)
    with open(src_path, "rb") as src, open(dst_path, "ab") as dst:
        while chunk := src.read(1 << 20):
            dst.write(chunk)
        dst.flush()
        os.fsync(dst.fileno())


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def recover_orphans():
    """
    Persist the RAM buffers of loggers whose process is no longer alive (killed or crashed)
    and delete them. Returns the number of recovered files. Never raises.
    """
    recovered = 0
    root = buffer_dir()
    try:
        runs = [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
    except OSError:
        return 0
    for run in runs:
        run_buf = os.path.join(root, run)
        try:
            names = os.listdir(run_buf)
        except OSError:
            continue
        for name in names:
            # <node>.<pid>.jsonl
            parts = name.rsplit(".", 2)
            if len(parts) != 3 or parts[2] != "jsonl" or not parts[1].isdigit():
                continue
            if _pid_alive(int(parts[1])):
                continue
            src = os.path.join(run_buf, name)
            try:
                if os.path.getsize(src) > 0:
                    _append_file(src, os.path.join(log_dir(), run, parts[0] + ".jsonl"))
                os.remove(src)
                recovered += 1
            except OSError:
                pass
        try:
            os.rmdir(run_buf)  # only succeeds when empty
        except OSError:
            pass
    return recovered


def _to_jsonable(value):
    """json.dumps fallback: numpy arrays/scalars and anything else via str()."""
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    return str(value)


class RunLogger:
    """Asynchronous, buffered, non-blocking per-run JSON-lines logger for one node."""

    LEVELS = ("DEBUG", "INFO", "WARN", "ERROR")

    def __init__(self, node, name=None):
        """
        node: the rclpy Node that owns the logger (used for its name, ROS clock and parameters).
        name: file name override (default: the node's fully qualified name, '/' -> '.').

        ROS parameters (declared on the node):
          run_log_enabled (bool, True)       disable to make every call a no-op
          run_log_min_level (str, "DEBUG")   drop entries below this level
          run_log_queue_size (int, 10000)    max queued entries before dropping the oldest
          run_log_persist_period (double, 2.0)  [s] how often the buffer is persisted
          run_log_max_buffer_mb (double, 64.0)  max unpersisted RAM buffer before dropping
        """
        self._node = node
        self.enabled = node.declare_parameter("run_log_enabled", True).value
        min_level = node.declare_parameter("run_log_min_level", "DEBUG").value.upper()
        queue_size = node.declare_parameter("run_log_queue_size", 10000).value
        self._persist_period = node.declare_parameter("run_log_persist_period", 2.0).value
        self._max_buffer_bytes = int(node.declare_parameter("run_log_max_buffer_mb", 64.0).value * 1e6)
        self._min_level = self.LEVELS.index(min_level) if min_level in self.LEVELS else 0

        self.node_name = name or node.get_fully_qualified_name().strip("/").replace("/", ".")
        self.run_id = None
        self._queue = collections.deque(maxlen=queue_size)
        self._dropped_queue = 0  # entries pushed out of the full queue
        self._dropped_io = 0  # entries lost to I/O errors / full RAM buffer
        self._io_errors = 0
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._closed = False
        self._thread = None
        if not self.enabled:
            return

        try:
            self.run_id = current_run_id()
            recover_orphans()
            run_buf = os.path.join(buffer_dir(), self.run_id)
            os.makedirs(run_buf, exist_ok=True)
            self._buffer_path = os.path.join(run_buf, f"{self.node_name}.{os.getpid()}.jsonl")
            self._persist_path = os.path.join(log_dir(), self.run_id, f"{self.node_name}.jsonl")
        except OSError as e:
            node.get_logger().error(f"RunLogger disabled, cannot set up log directories: {e}")
            self.enabled = False
            return

        self._thread = threading.Thread(target=self._run, name="run_logger", daemon=True)
        self._thread.start()
        atexit.register(self.close)
        try:
            node.context.on_shutdown(self.close)
        except Exception:  # older rclpy without on_shutdown: atexit still covers normal exits
            pass
        self.info("run_logger_started", run_id=self.run_id, pid=os.getpid(), file=self._persist_path)

    # ------------------------------------------------------------------ public API

    def log(self, level, event, **fields):
        """Queue one entry. Never blocks or raises. level: DEBUG/INFO/WARN/ERROR."""
        if not self.enabled or self._closed:
            return
        try:
            lvl = self.LEVELS.index(level)
        except ValueError:
            lvl = 1
        if lvl < self._min_level:
            return
        try:
            t = self._node.get_clock().now().nanoseconds * 1e-9
        except Exception:
            t = None
        if len(self._queue) == self._queue.maxlen:
            self._dropped_queue += 1
        self._queue.append((t, time.time(), self.LEVELS[lvl], event, fields))

    def debug(self, event, **fields):
        self.log("DEBUG", event, **fields)

    def info(self, event, **fields):
        self.log("INFO", event, **fields)

    def warn(self, event, **fields):
        self.log("WARN", event, **fields)

    def error(self, event, **fields):
        self.log("ERROR", event, **fields)

    def flush(self):
        """Ask the background thread to write and persist now (non-blocking)."""
        self._wake.set()

    def close(self):
        """Stop the background thread after a final write + persist. Safe to call twice."""
        if self._closed:
            return
        self._closed = True
        if self._thread is not None:
            self._stop.set()
            self._wake.set()
            self._thread.join(timeout=5.0)

    @property
    def stats(self):
        return {"dropped_queue": self._dropped_queue, "dropped_io": self._dropped_io,
                "io_errors": self._io_errors, "queued": len(self._queue)}

    # ------------------------------------------------------------------ background thread

    def _run(self):
        last_persist = time.monotonic()
        while True:
            self._wake.wait(timeout=self._persist_period)
            self._wake.clear()
            stopping = self._stop.is_set()
            self._write_queued()
            if stopping or time.monotonic() - last_persist >= self._persist_period:
                self._report_drops()
                self._write_queued()
                self._persist()
                last_persist = time.monotonic()
            if stopping:
                break
        # Remove the buffer file if everything was persisted, else leave it for recovery
        try:
            if os.path.getsize(self._buffer_path) == 0:
                os.remove(self._buffer_path)
        except OSError:
            pass

    def _report_drops(self):
        if self._dropped_queue or self._dropped_io:
            dq, di = self._dropped_queue, self._dropped_io
            self._dropped_queue = self._dropped_io = 0
            self._queue.append((None, time.time(), "WARN", "run_logger_dropped",
                                {"dropped_queue": dq, "dropped_io": di, "io_errors": self._io_errors}))

    def _write_queued(self):
        """Serialize all queued entries and append them to the RAM buffer file."""
        if not self._queue:
            return
        lines = []
        while self._queue:
            t, wall, level, event, fields = self._queue.popleft()
            entry = {"t": t, "wall": wall, "node": self.node_name, "level": level, "event": event}
            entry.update(fields)
            try:
                lines.append(json.dumps(entry, default=_to_jsonable))
            except (TypeError, ValueError) as e:
                lines.append(json.dumps({"t": t, "wall": wall, "node": self.node_name, "level": "ERROR",
                                         "event": "run_logger_serialization_error", "for_event": event,
                                         "error": str(e)}))
        data = ("\n".join(lines) + "\n").encode()
        try:
            if os.path.exists(self._buffer_path) and \
                    os.path.getsize(self._buffer_path) + len(data) > self._max_buffer_bytes:
                self._dropped_io += len(lines)
                return
            with open(self._buffer_path, "ab") as f:
                f.write(data)
        except OSError:
            self._io_errors += 1
            self._dropped_io += len(lines)

    def _persist(self):
        """Append the RAM buffer to the persistent file, then clear the buffer."""
        try:
            if not os.path.exists(self._buffer_path) or os.path.getsize(self._buffer_path) == 0:
                return
            _append_file(self._buffer_path, self._persist_path)
            with open(self._buffer_path, "wb"):
                pass  # truncate: only the unpersisted tail stays in RAM
        except OSError:
            self._io_errors += 1  # keep the buffer, retry next period


def main():
    """CLI used by kill_ros_nodes.sh: --recover persists orphaned buffers, --end-run ends the run."""
    args = sys.argv[1:]
    if not args or "-h" in args or "--help" in args:
        print("usage: python3 -m all_seaing_common.run_logger [--recover] [--end-run]")
        return
    if "--recover" in args:
        n = recover_orphans()
        print(f"run_logger: recovered {n} orphaned log buffer(s) into {log_dir()}")
    if "--end-run" in args:
        end_run()
        print("run_logger: current run ended")


if __name__ == "__main__":
    main()
