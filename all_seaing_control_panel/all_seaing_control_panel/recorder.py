"""
Laptop-side recorder (Recordings tab): records the boat's data **through foxglove_bridge**
(nothing extra runs on the boat besides the idle-unless-used image keyframe compressor) into
rolling MCAP segments, from which time ranges are saved as standalone MCAP recordings
(openable in Foxglove, `ros2 bag play`, and the Recordings viewer).

What is recorded (config/recordings.yaml):
- `topics`: written as received (light topics: boxes, /run_log, /diagnostics, TF, maps, odometry,
  camera info, and the JPEG fused images that bbox_project_pcloud publishes while subscribed).
- `keyframe_cloud_topic` (the full filtered LiDAR cloud): received continuously but **only written
  for fused keyframes**: the scans bbox_project_pcloud fused for images with at least one YOLO box
  (its /run_log `frame` events carry their `cloud_stamp` and `n_boxes`), each scan once. The image
  keyframes are the same frames (bbox_project_pcloud fused_image/<camera>/compressed).
- `latched_topics` (e.g. /tf_static, camera_info): also re-written at the start of every segment
  (and of saved recordings), so each file is self-contained.

Files: <buffer_dir>/<session>/<YYYYmmdd-HHMMSS>.mcap, one per `segment_s`; a session
(<YYYYmmdd-HHMMSS> of start()) is one start..stop, shown as its own source in the Recordings tab.
Segments older than `buffer_s` are deleted, across all sessions (empty sessions are removed). Log time = laptop receive time [ns]; publish time = bridge receive time.
"""

import collections
import glob
import json
import os
import threading
import time

from mcap.writer import CompressionType, Writer

from all_seaing_control_panel.bridge import BridgeClient, decode_string_cdr, header_stamp_ns


class Segment:
    """One MCAP file being written."""

    def __init__(self, path):
        self.path = path
        self.f = open(path, "wb")
        self.w = Writer(self.f, compression=CompressionType.ZSTD)
        self.w.start(profile="ros2", library="arcturus_control_panel")
        self.schemas, self.channels = {}, {}

    def channel(self, ch):
        """MCAP channel id for a bridge channel {topic, schemaName, schema, schemaEncoding, encoding}."""
        if ch["topic"] not in self.channels:
            key = ch["schemaName"]
            if key not in self.schemas:
                self.schemas[key] = self.w.register_schema(
                    name=ch["schemaName"], encoding=ch.get("schemaEncoding") or "ros2msg", data=ch["schema"].encode())
            self.channels[ch["topic"]] = self.w.register_channel(
                topic=ch["topic"], message_encoding=ch.get("encoding") or "cdr", schema_id=self.schemas[key])
        return self.channels[ch["topic"]]

    def write(self, ch, log_time, data, publish_time=None):
        self.w.add_message(channel_id=self.channel(ch), log_time=log_time, data=data,
                           publish_time=publish_time or log_time)

    def close(self):
        self.w.finish()
        self.f.close()


class Recorder:
    def __init__(self, url, cfg):
        self.url = url
        self.cfg = cfg
        self.buffer_dir = os.path.expanduser(cfg["buffer_dir"])
        os.makedirs(self.buffer_dir, exist_ok=True)
        self.topics = set(cfg["topics"]) | set(cfg.get("latched_topics", []))
        self.cloud_topic = cfg.get("keyframe_cloud_topic")
        self.enabled = False
        self._client = BridgeClient(url, lambda t: t in self.topics or t == self.cloud_topic, self._on_message,
                                    name="recorder")
        self._lock = threading.Lock()
        self._seg = None
        self._seg_start = 0
        self._session_dir = self.buffer_dir
        self._latched = {}  # topic -> (channel, data)
        self._clouds = collections.OrderedDict()  # header stamp ns -> (channel, data, log_time)
        self._saved_clouds = collections.deque(maxlen=50)  # stamps of recently written keyframe scans
        # fused-image keyframes: at most keyframe_max_rate per camera topic; their stamps pick the scans
        self.keyframe_topics = {t for c in cfg.get("cameras", {}).values()
                                for t in (c["keyframes"] if isinstance(c["keyframes"], list) else [c["keyframes"]])}
        self.keyframe_rate = float(cfg.get("keyframe_max_rate", 5.0))
        self._last_keyframe = {}
        self._kept_images = collections.deque(maxlen=200)
        self.stats = collections.Counter()  # topic -> messages written
        self.bytes = 0

    # ------------------------------------------------------------------ control

    def start(self):
        if self.enabled:
            return
        self.enabled = True
        self._session_dir = os.path.join(self.buffer_dir, time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(self._session_dir, exist_ok=True)
        self._client.start()

    def stop(self):
        self.enabled = False
        self._client.stop()
        with self._lock:
            if self._seg:
                self._seg.close()
                self._seg = None

    def current_segment(self):
        """Path of the segment being written, or None."""
        seg = self._seg
        return seg.path if seg else None

    def status(self):
        segs = glob.glob(os.path.join(self.buffer_dir, "*.mcap")) + glob.glob(os.path.join(self.buffer_dir, "*", "*.mcap"))
        size = sum(os.path.getsize(s) for s in segs)
        return {"enabled": self.enabled, "connected": self._client.connected, "error": self._client.error, "url": self.url,
                "segments": len(segs), "buffer_mb": round(size / 1e6, 1), "written_mb": round(self.bytes / 1e6, 1),
                "messages": dict(self.stats), "buffer_dir": self.buffer_dir}

    # ------------------------------------------------------------------ writing

    def _segment(self, now_ns):
        if self._seg is None or now_ns - self._seg_start > self.cfg.get("segment_s", 60) * 1e9:
            if self._seg:
                self._seg.close()
            name = time.strftime("%Y%m%d-%H%M%S", time.localtime(now_ns / 1e9)) + ".mcap"
            self._seg = Segment(os.path.join(self._session_dir, name))
            self._seg_start = now_ns
            for ch, data in self._latched.values():  # self-contained segments
                self._seg.write(ch, now_ns, data)
            self._prune(now_ns)
        return self._seg

    def _prune(self, now_ns):
        """Delete segments (of any session) last written more than buffer_s ago, and empty sessions."""
        cutoff = now_ns / 1e9 - self.cfg.get("buffer_s", 1800)
        current = os.path.abspath(self._seg.path) if self._seg else None
        for path in glob.glob(os.path.join(self.buffer_dir, "*.mcap")) + glob.glob(os.path.join(self.buffer_dir, "*", "*.mcap")):
            if os.path.abspath(path) != current and os.path.getmtime(path) < cutoff:
                os.remove(path)
        for d in glob.glob(os.path.join(self.buffer_dir, "*/")):
            if os.path.abspath(d.rstrip("/")) != os.path.abspath(self._session_dir) and not os.listdir(d):
                os.rmdir(d)

    def _write(self, ch, log_time, data, publish_time):
        seg = self._segment(log_time)
        seg.write(ch, log_time, data, publish_time)
        self.stats[ch["topic"]] += 1
        self.bytes += len(data)

    def _on_message(self, ch, bridge_ts, data):
        now = time.time_ns()
        topic = ch["topic"]
        with self._lock:
            if not self.enabled:
                return
            if topic == self.cloud_topic:
                # keep recent scans; written only when /run_log says they were fused (keyframes)
                self._clouds[header_stamp_ns(data)] = (ch, data, now)
                while len(self._clouds) > 100:
                    self._clouds.popitem(last=False)
                return
            if topic in self.cfg.get("latched_topics", []):
                self._latched[topic] = (ch, data)
            if topic in self.keyframe_topics:  # fused images: <= keyframe_max_rate per camera (receive time)
                if now - self._last_keyframe.get(topic, 0) < 1e9 / self.keyframe_rate:
                    return
                self._last_keyframe[topic] = now
                self._kept_images.append(header_stamp_ns(data))
            self._write(ch, now, data, bridge_ts)
            if topic == "/run_log":
                self._keyframe_clouds(decode_string_cdr(data), now)

    def _keyframe_clouds(self, text, now):
        for line in text.splitlines():
            if '"event": "frame"' not in line or "cloud_stamp" not in line:
                continue
            try:
                e = json.loads(line)
                stamp = round(e["cloud_stamp"] * 1e9)
            except (ValueError, KeyError):
                continue
            if e.get("n_boxes", 0) == 0:
                continue  # no keyframe for an image without boxes
            img = round(e.get("image_stamp", -1) * 1e9)
            if not any(abs(img - k) < 1e6 for k in self._kept_images):
                continue  # its image wasn't kept (keyframe rate cap): no scan either
            match = min(self._clouds, key=lambda k: abs(k - stamp), default=None)
            if match is None or abs(match - stamp) > 1e6:  # 1 ms: same scan
                continue
            if match in self._saved_clouds:  # several cameras fuse the same scan: write it once
                continue
            ch, data, log_time = self._clouds[match]
            self._saved_clouds.append(match)
            self._write(ch, log_time, data, log_time)
