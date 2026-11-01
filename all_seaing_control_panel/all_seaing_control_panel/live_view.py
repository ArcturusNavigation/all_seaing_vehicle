"""
Live frame view (Recordings tab, "Live view"): the same view as the recordings' frame viewer (camera
keyframe + YOLO boxes colored by fusion outcome + fused LiDAR scan projected into the image + top
view), built from the latest data received **through foxglove_bridge**, without recording anything.

Topics (config/recordings.yaml): per camera the first `keyframes` topic (fusion's fused_image: every
image bbox_project_pcloud processed with YOLO boxes), `boxes`, `camera_info`; /run_log (fusion
outcomes, matched to the image by `image_stamp`); `keyframe_cloud_topic` (the fused scan is the one
whose stamp the `frame` event's `cloud_stamp` names); /tf and /tf_static.

The bridge subscription runs only while the view is being polled: it stops `idle_s` seconds after
the last request, so the boat's fused-image JPEG encoding (only while subscribed) and the
Wi-Fi link are idle otherwise.

latest(camera) returns the newest keyframe whose fusion `frame` event has arrived (so its outcomes
and scan are known); images fusion didn't process (yet) are never shown.
"""

import collections
import json
import threading
import time
from types import SimpleNamespace

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from all_seaing_control_panel.bridge import BridgeClient, decode_string_cdr, header_stamp_ns
from all_seaing_control_panel.recordings import TfGraph, build_frame, keyframe_topics, log_camera


class LiveView:
    def __init__(self, url, cfg, idle_s=20.0):
        self.cameras = cfg["cameras"]
        self.cloud_topic = cfg.get("keyframe_cloud_topic")
        self.idle_s = idle_s
        self._by_topic = {}  # topic -> (camera, kind)
        for name, c in self.cameras.items():
            self._by_topic[keyframe_topics(c)[0]] = (name, "keyframes")
            for kind in ("boxes", "camera_info"):
                self._by_topic[c[kind]] = (name, kind)
        self._by_log_camera = {log_camera(self.cameras, n): n for n in self.cameras}
        topics = set(self._by_topic) | {"/run_log", "/tf", "/tf_static", self.cloud_topic}
        self._client = BridgeClient(url, lambda t: t in topics, self._on_message, name="live_view")
        self._lock = threading.Lock()
        self._last_request = 0.0
        self._reset()
        threading.Thread(target=self._idle_watch, daemon=True, name="live_view idle").start()

    def _reset(self):
        # images wait here for their fusion `frame` event: /run_log is published in batches, ~1-3 s late
        self._images = {c: collections.deque(maxlen=150) for c in self.cameras}  # (stamp ns, recv time, schema, data)
        self._boxes = {c: collections.OrderedDict() for c in self.cameras}  # stamp ns -> (schema, data)
        self._info = {}  # camera -> (schema, data)
        self._events = {c: collections.OrderedDict() for c in self.cameras}  # image stamp ns -> [events]
        self._frames = {c: {} for c in self.cameras}  # image stamp ns -> frame event
        self._clouds = collections.OrderedDict()  # stamp ns -> (schema, data)
        self._tf_static = TfGraph()
        self._tf = {}  # (parent, child) -> latest dynamic TransformStamped
        self._built = {}  # camera -> (key, frame): the last built frame, reused while nothing changes

    # ------------------------------------------------------------------ subscription lifecycle

    def _idle_watch(self):
        while True:
            time.sleep(2.0)
            if self._client.enabled and time.time() - self._last_request > self.idle_s:
                self._client.stop()

    def status(self):
        return {"active": self._client.enabled, "connected": self._client.connected, "error": self._client.error}

    # ------------------------------------------------------------------ receiving

    @staticmethod
    def _trim(d, n=60):
        while len(d) > n:
            d.popitem(last=False)

    def _on_message(self, ch, receive_ns, data):
        topic, schema = ch["topic"], ch["schemaName"]
        with self._lock:
            if topic in self._by_topic:
                cam, kind = self._by_topic[topic]
                if kind == "keyframes":
                    self._images[cam].append((header_stamp_ns(data), time.time(), schema, data))
                elif kind == "boxes":
                    self._boxes[cam][header_stamp_ns(data)] = (schema, data)
                    self._trim(self._boxes[cam], 150)
                else:
                    self._info[cam] = (schema, data)
            elif topic == self.cloud_topic:
                self._clouds[header_stamp_ns(data)] = (schema, data)
                self._trim(self._clouds, 100)  # ~5-10 s, like the images
            elif topic == "/tf_static":
                self._tf_static.add(deserialize_message(data, get_message(schema)))
            elif topic == "/tf":
                for t in deserialize_message(data, get_message(schema)).transforms:
                    self._tf[(t.header.frame_id, t.child_frame_id)] = t
            elif topic == "/run_log":
                for line in decode_string_cdr(data).splitlines():
                    if '"image_stamp"' not in line:
                        continue
                    try:
                        e = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    cam = self._by_log_camera.get(e.get("camera"))  # run-log camera name -> camera key
                    if cam is None:
                        continue
                    stamp = round(e["image_stamp"] * 1e9)
                    if e.get("event") == "frame":
                        self._frames[cam][stamp] = e
                        if len(self._frames[cam]) > 200:
                            self._frames[cam].pop(next(iter(self._frames[cam])))
                    else:
                        evs = self._events[cam].setdefault(stamp, [])
                        if evs and abs(e.get("wall", 0) - evs[-1].get("wall", 0)) > 2.0:
                            evs.clear()  # same image stamp again: a looping rosbag's next pass
                        evs.append(e)
                        self._trim(self._events[cam], 200)

    # ------------------------------------------------------------------ the view

    @staticmethod
    def _near(d, stamp, tol_ns):
        """Value of the key of d nearest to stamp within tol_ns, else None."""
        k = min(d, key=lambda k: abs(k - stamp), default=None)
        return d[k] if k is not None and abs(k - stamp) <= tol_ns else None

    def latest(self, camera):
        """The current frame of a camera (see module doc), or {"waiting": reason}."""
        self._last_request = time.time()
        if not self._client.enabled:
            self._reset()
            self._client.start()
        with self._lock:
            images = list(self._images[camera])
            if not images:
                return {"waiting": "no keyframes received yet" if self._client.connected
                        else f"connecting to the bridge {self._client.error}".strip()}
            # newest image already fused (outcomes and scan known)
            chosen = next((im for im in reversed(images) if self._near(self._frames[camera], im[0], 1000)), None)
            if chosen is None:
                return {"waiting": "no fused frame yet (is bbox_project_pcloud running?)"}
            stamp, recv, img_schema, img_data = chosen
            boxes = self._near(self._boxes[camera], stamp, 1000)
            events = list(self._near(self._events[camera], stamp, 1000) or [])
            frame_event = self._near(self._frames[camera], stamp, 1000)
            cloud = None
            if frame_event and "cloud_stamp" in frame_event:
                cloud = self._near(self._clouds, round(frame_event["cloud_stamp"] * 1e9), 1_000_000)
            info = self._info.get(camera)
            tf = TfGraph()
            tf.edges.update(self._tf_static.edges)
            tf.add(SimpleNamespace(transforms=list(self._tf.values())))
        key = (camera, stamp, frame_event is not None, len(events), cloud is not None)
        cached = self._built.get(camera)
        if cached and cached[0] == key:  # nothing new since the last poll: no rebuild
            out = dict(cached[1])
            out["age_s"] = round(time.time() - recv, 2)
            return out
        des = lambda sd: deserialize_message(sd[1], get_message(sd[0])) if sd else None  # noqa: E731
        out = build_frame(camera, des((img_schema, img_data)), des(boxes), events, frame_event, [des(cloud)],
                          des(info), tf, self.cameras[camera].get("camera_convention", "optical"))
        out.update({"stamp": stamp / 1e9, "time": recv, "age_s": round(time.time() - recv, 2),
                    "fused": frame_event is not None})
        if cloud is None and "cloud_stamp" in frame_event:
            out["lidar_note"] = "fused scan not received over the bridge"
        self._built[camera] = (key, out)
        return out

