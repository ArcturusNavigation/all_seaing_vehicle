"""
Reading recordings (Recordings tab): the rolling buffer written by recorder.py and saved MCAP
recordings (or any MCAP with the same topics, e.g. a ros2 bag recorded as MCAP).

Provides:
- sources():                 the buffer + saved recordings, with time spans
- timeline(source):          per-second activity for the timeline (fusion detections/rejections,
                             keyframes per camera, WARN/ERROR log entries)
- save(t0, t1, name):        copy the buffer's messages in [t0, t1] (+ latched topics) to a saved MCAP
- frames(source, camera):    the image keyframes (time + header stamp)
- frame(source, camera, i):  everything at one keyframe: JPEG image, YOLO boxes with their fusion
                             outcome (from /run_log, matched by image stamp), the fused LiDAR scan
                             projected into the image (recorded camera_info + TF), a top-down view
                             in base_link, and the log entries around it
- run_log_entries(source):   the /run_log entries, for the analyzers
Times are log times (laptop receive time) in seconds unless noted.
"""

import base64
import bisect
import glob
import json
import os
import re
import time

import numpy as np
from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from all_seaing_control_panel.bridge import decode_string_cdr, header_stamp_ns

BUFFER = "__buffer__"  # buffer files from before recording sessions (flat in buffer_dir)
SESSION = "__session__"  # + session folder name: one recorder start..stop (recorder.py)


def keyframe_topics(cam):
    """A camera's keyframe (JPEG) topics: one name or a list (fused images, then older names)."""
    k = cam["keyframes"]
    return list(k) if isinstance(k, (list, tuple)) else [k]


def log_camera(cameras, camera):
    """A camera's name in fusion's run log (bbox_project_pcloud camera_name; "" in sim)."""
    return cameras[camera].get("log_camera", camera)


def is_buffer(source):
    """A source in the rolling buffer (a session or the older flat files), as opposed to a saved recording."""
    return source == BUFFER or source.startswith(SESSION)


def _quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tf_matrix(t):
    m = np.eye(4)
    m[:3, :3] = _quat_to_mat(t.transform.rotation)
    m[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return m


class TfGraph:
    """Transforms between frames from TFMessages (parent->child edges, both directions)."""

    def __init__(self):
        self.edges = {}  # (parent, child) -> 4x4 (child -> parent)

    def add(self, tf_msg):
        for t in tf_msg.transforms:
            self.edges[(t.header.frame_id.lstrip("/"), t.child_frame_id.lstrip("/"))] = _tf_matrix(t)

    def lookup(self, target, source):
        """4x4 mapping points in `source` to `target`, or None (BFS over the frame tree)."""
        target, source = target.lstrip("/"), source.lstrip("/")
        adj = {}
        for (p, c), m in self.edges.items():
            adj.setdefault(c, []).append((p, m))  # child -> parent
            adj.setdefault(p, []).append((c, np.linalg.inv(m)))  # parent -> child
        frontier, seen = [(source, np.eye(4))], {source}
        while frontier:
            frame, m = frontier.pop(0)
            if frame == target:
                return m
            for nxt, step in adj.get(frame, []):
                if nxt not in seen:
                    seen.add(nxt)
                    frontier.append((nxt, step @ m))
        return None


class Recordings:
    def __init__(self, cfg):
        self.cfg = cfg
        self.buffer_dir = os.path.expanduser(cfg["buffer_dir"])
        self.saved_dir = os.path.expanduser(cfg["saved_dir"])
        os.makedirs(self.saved_dir, exist_ok=True)
        self.cameras = cfg["cameras"]  # name -> {keyframes, boxes, camera_info}
        self._cache = {}

    # ------------------------------------------------------------------ files

    def files(self, source):
        if source == BUFFER:
            return sorted(glob.glob(os.path.join(self.buffer_dir, "*.mcap")))
        if source.startswith(SESSION):
            d = os.path.join(self.buffer_dir, os.path.basename(source[len(SESSION):]))
            if not os.path.isdir(d):
                raise FileNotFoundError(source)
            return sorted(glob.glob(os.path.join(d, "*.mcap")))
        path = os.path.join(self.saved_dir, os.path.basename(source))
        if not os.path.exists(path):
            raise FileNotFoundError(source)
        return [path]

    def _iter(self, source, topics=None, t0=None, t1=None):
        """(topic, schema name, log_time ns, data) for the source's messages, in log-time order per file."""
        for path in self.files(source):
            try:
                with open(path, "rb") as f:
                    reader = make_reader(f)
                    for schema, channel, msg in reader.iter_messages(
                            topics=topics, start_time=int(t0 * 1e9) if t0 else None,
                            end_time=int(t1 * 1e9) if t1 else None, log_time_order=True):
                        yield channel.topic, schema.name if schema else "", msg.log_time, msg.data
            except Exception:
                continue  # segment being written (no footer yet) or truncated

    def _key(self, source):
        return tuple((p, os.path.getsize(p)) for p in self.files(source))

    def sources(self, active_segments=()):
        """Recording sessions in the buffer (newest first; `active` if a recorder writes into it),
        the older flat buffer files, then the saved recordings."""
        out = []
        active_dirs = {os.path.dirname(os.path.abspath(p)) for p in active_segments}
        # newest first by modification time (folder names are local time: not monotonic across DST changes)
        for d in sorted(glob.glob(os.path.join(self.buffer_dir, "*/")), key=os.path.getmtime, reverse=True):
            d = d.rstrip("/")
            files = sorted(glob.glob(os.path.join(d, "*.mcap")))
            if not files:
                continue
            name = os.path.basename(d)
            try:
                when = time.strftime("%b %d, %H:%M:%S", time.strptime(name, "%Y%m%d-%H%M%S"))
            except ValueError:
                when = name
            end = time.strftime("%H:%M:%S", time.localtime(max(os.path.getmtime(f) for f in files)))
            active = os.path.abspath(d) in active_dirs
            out.append({"id": SESSION + name, "name": f"Session {when} – {'now' if active else end}", "files": len(files),
                        "active": active, "buffer": True,
                        "size_mb": round(sum(os.path.getsize(p) for p in files) / 1e6, 1)})
        buf = self.files(BUFFER)
        if buf:
            out.append({"id": BUFFER, "name": "Older buffer (before sessions)", "files": len(buf), "buffer": True,
                        "size_mb": round(sum(os.path.getsize(p) for p in buf) / 1e6, 1)})
        for p in sorted(glob.glob(os.path.join(self.saved_dir, "*.mcap")), key=os.path.getmtime, reverse=True):
            out.append({"id": os.path.basename(p), "name": os.path.basename(p)[:-5], "files": 1,
                        "size_mb": round(os.path.getsize(p) / 1e6, 1)})
        return out

    # ------------------------------------------------------------------ timeline

    def run_log_entries(self, source, t0=None, t1=None):
        """/run_log entries with _rec_time (log time [s]) added."""
        out = []
        for _, _, log_time, data in self._iter(source, ["/run_log"], t0, t1):
            for line in decode_string_cdr(data).splitlines():
                try:
                    e = json.loads(line)
                except json.JSONDecodeError:
                    continue
                e["_rec_time"] = log_time / 1e9
                out.append(e)
        return out

    def timeline(self, source, target_buckets=600, min_bucket_s=0.1):
        """Activity over time for the timeline: counts per bucket of `bucket_s` seconds (about
        `target_buckets` over the recording, at least `min_bucket_s`). Images and scans are placed at
        their receive (log) time; /run_log entries at their own `wall` time (the node's clock),
        shifted onto the log clock with the batch they arrived in (the newest entry of a batch was
        published just before it was received), since run logs are published in batches."""
        key = ("timeline", self._key(source))
        if key in self._cache:
            return self._cache[key]
        kf_topics = {t: name for name, c in self.cameras.items() for t in keyframe_topics(c)}
        cloud_topic = self.cfg.get("keyframe_cloud_topic")
        items = []  # (time, field)
        for topic, _, log_time, data in self._iter(source, list(kf_topics) + ["/run_log", cloud_topic]):
            t = log_time / 1e9
            if topic in kf_topics:
                items.append((t, f"kf_{kf_topics[topic]}"))
            elif topic == cloud_topic:
                items.append((t, "clouds"))
            else:
                batch = []
                for line in decode_string_cdr(data).splitlines():
                    ev = re.search(r'"event": "(\w+)"', line)
                    lvl = re.search(r'"level": "(\w+)"', line)
                    wall = re.search(r'"wall": ([\d.]+)', line)
                    field = {"box_detected": "detections", "box_rejected": "rejected", "box_skipped": "skipped"}.get(
                        ev.group(1) if ev else "")
                    level = {"WARN": "warn", "ERROR": "error"}.get(lvl.group(1) if lvl else "")
                    w = float(wall.group(1)) if wall else None
                    batch += [(w, f) for f in (field, level) if f]
                walls = [w for w, _ in batch if w is not None]
                offset = t - max(walls) if walls else 0.0
                items += [((w + offset) if w is not None else t, f) for w, f in batch]
        if not items:
            res = {"start": None, "end": None, "bucket_s": 1.0, "seconds": [], "cameras": list(self.cameras)}
            self._cache[key] = res
            return res
        t0, t1 = min(i[0] for i in items), max(i[0] for i in items)
        bucket = max(min_bucket_s, (t1 - t0) / target_buckets)
        bucket = min((b for b in (0.1, 0.2, 0.5, 1, 2, 5, 10, 30, 60) if b >= bucket), default=bucket)  # round
        buckets = {}
        empty = {"detections": 0, "rejected": 0, "skipped": 0, "warn": 0, "error": 0,
                 **{f"kf_{n}": 0 for n in self.cameras}, "clouds": 0}
        for t, f in items:
            k = int((t - t0) // bucket)
            buckets.setdefault(k, dict(empty))[f] += 1
        ks = sorted(buckets)
        res = {"start": t0, "end": t0 + (ks[-1] + 1) * bucket, "bucket_s": bucket,
               "seconds": [{"t": t0 + k * bucket, **buckets[k]} for k in ks], "cameras": list(self.cameras)}
        self._cache[key] = res
        return res

    # ------------------------------------------------------------------ save a range

    def save(self, t0, t1, name, source=BUFFER):
        name = re.sub(r"[^\w.-]+", "_", name).strip("_") or "recording"
        path = os.path.join(self.saved_dir, name + ".mcap")
        if os.path.exists(path):
            raise FileExistsError(f"{name} already exists")
        latched = set(self.cfg.get("latched_topics", []))
        last_latched = {}
        with open(path, "wb") as f:
            w = Writer(f, compression=CompressionType.ZSTD)
            w.start(profile="ros2", library="arcturus_control_panel")
            schemas, channels, count = {}, {}, 0

            def chan(schema, channel):
                if channel.topic not in channels:
                    if schema.name not in schemas:
                        schemas[schema.name] = w.register_schema(schema.name, schema.encoding, schema.data)
                    channels[channel.topic] = w.register_channel(channel.topic, channel.message_encoding,
                                                                 schemas[schema.name])
                return channels[channel.topic]

            for p in self.files(source):
                try:
                    with open(p, "rb") as rf:
                        for schema, channel, msg in make_reader(rf).iter_messages(log_time_order=True):
                            t = msg.log_time / 1e9
                            if channel.topic in latched and t < t0:
                                last_latched[channel.topic] = (schema, channel, msg)
                            if t0 <= t <= t1:
                                if channel.topic in latched and channel.topic in last_latched:
                                    last_latched.pop(channel.topic)
                                w.add_message(chan(schema, channel), msg.log_time, msg.data, msg.publish_time)
                                count += 1
                except Exception:
                    continue
            for schema, channel, msg in last_latched.values():  # calibration/static TF from before the range
                w.add_message(chan(schema, channel), int(t0 * 1e9), msg.data, msg.publish_time)
            w.finish()
        return {"saved": os.path.basename(path), "messages": count, "size_mb": round(os.path.getsize(path) / 1e6, 1)}

    def delete(self, source, keep=()):
        """Delete a saved recording, or clear the buffer (except the files in `keep`: segments being written)."""
        keep = {os.path.abspath(k) for k in keep}
        removed = 0
        for path in self.files(source):
            if os.path.abspath(path) not in keep:
                os.remove(path)
                removed += 1
        if source.startswith(SESSION) and not keep & {os.path.abspath(p) for p in self.files(source)}:
            os.rmdir(os.path.join(self.buffer_dir, os.path.basename(source[len(SESSION):])))
        self._cache.clear()
        return removed

    # ------------------------------------------------------------------ frames

    def frames(self, source, camera):
        """[(log time, image stamp)] of a camera's keyframes."""
        key = ("frames", camera, self._key(source))
        if key not in self._cache:
            topics = keyframe_topics(self.cameras[camera])
            self._cache[key] = [(lt / 1e9, header_stamp_ns(d) / 1e9)
                                for _, _, lt, d in self._iter(source, topics)]
        return self._cache[key]

    def frame_list(self, source, camera):
        """Keyframes with their fusion outcome counts (for filtering), matched by image stamp and log time."""
        key = ("frame_list", camera, self._key(source))
        if key in self._cache:
            return self._cache[key]
        counts = {}  # (rounded stamp, rounded log time) -> Counter-like dict; log time disambiguates looping bags
        for e in self.run_log_entries(source):
            if e.get("camera") != log_camera(self.cameras, camera) or "image_stamp" not in e:
                continue
            ev = e.get("event")
            name = {"box_detected": "detected", "box_skipped": "skipped"}.get(ev, e.get("reason") if ev == "box_rejected" else None)
            if name:
                c = counts.setdefault(round(e["image_stamp"], 6), [])
                c.append((e["_rec_time"], name))
        out = []
        from_bag = not counts  # no run log: fused = fusion published an output for the image
        for i, (t, stamp) in enumerate(self.frames(source, camera)):
            c = {}
            for rt, name in counts.get(round(stamp, 6), []):
                if -1.0 <= rt - t <= 6.0:  # same pass of the data (not another loop of a replayed bag), as in frame()
                    c[name] = c.get(name, 0) + 1
            if from_bag:
                o = self._fusion_output_for(source, camera, stamp, t)
                if o is not None:
                    c = {"fused": 1, **({"detected": o[2]} if o[2] else {})}
            out.append({"index": i, "time": t, "stamp": stamp, "counts": c})
        self._cache[key] = out
        return out

    def _latest(self, source, topic, schema_name):
        """Last message of a latched topic (/tf_static, camera_info) anywhere in the source (cached)."""
        key = ("latest", topic, source, tuple(self.files(source)))  # not invalidated as the buffer grows
        if key not in self._cache:
            msg = None
            for _, _, _, d in self._iter(source, [topic]):
                msg = d
            self._cache[key] = deserialize_message(msg, get_message(schema_name)) if msg else None
        return self._cache[key]

    # ------------------------------------------------------------------ fusion outputs (no run log)

    def _fusion_outputs(self, source, camera):
        """Fusion's own per-frame outputs for a camera (recordings without /run_log, e.g. imported
        rosbags): sorted [(scan stamp s, log time s, n obstacles, ObstacleMap)] from its `detections`
        topic. bbox_project_pcloud stamps its outputs with the stamp of the scan it fused."""
        cam = self.cameras[camera]
        key = ("fusion_outputs", camera, self._key(source))
        if key not in self._cache:
            out = []
            if cam.get("detections"):
                for _, schema, lt, data in self._iter(source, [cam["detections"]]):
                    m = deserialize_message(data, get_message(schema))
                    st = m.header.stamp if m.header.stamp.sec else m.local_header.stamp  # older code: local_header only
                    out.append((st.sec + st.nanosec * 1e-9, lt / 1e9, len(m.obstacles), m))
            out.sort(key=lambda o: o[0])
            self._cache[key] = out
        return self._cache[key]

    def _fusion_output_for(self, source, camera, stamp, log_t, window=0.15):
        """The fusion output for an image: the one whose scan is nearest the image stamp (fusion's
        approximate-time sync paired them), published within a few seconds after it."""
        outs = self._fusion_outputs(source, camera)
        i = bisect.bisect_left([o[0] for o in outs], stamp - window)
        cands = [o for o in outs[i:i + 20] if abs(o[0] - stamp) <= window and -1.0 <= o[1] - log_t <= 3.0]
        return min(cands, key=lambda o: abs(o[0] - stamp)) if cands else None

    def _bag_fusion_events(self, source, camera, stamp, log_t, out, boxes, image, info, tf):
        """box_detected / box_rejected events inferred from fusion's recorded outputs (no run log):
        detected = a detection of the box's label projects into the box; no_points = fusion
        published no point cloud for the box (it only does when the box has LiDAR points);
        not_detected_bag = it had points but gave no detection (the reason wasn't recorded)."""
        cam = self.cameras[camera]
        scan_stamp, det = out[0], out[3]
        objects = selected = None
        for topic, schema, _, data in self._iter(source, [cam.get("object_clouds"), cam.get("selected_clouds")],
                                                 log_t - 1.0, log_t + 3.0):
            if abs(header_stamp_ns(data) / 1e9 - scan_stamp) > 1e-6:
                continue
            if topic == cam.get("object_clouds"):
                objects = deserialize_message(data, get_message(schema))
            else:
                selected = deserialize_message(data, get_message(schema))
        frame_event = {"event": "frame", "camera": camera, "image_stamp": stamp, "cloud_stamp": scan_stamp,
                       "n_detections": len(det.obstacles), "source": "rosbag (fusion outputs)"}
        if boxes is None or image is None or info is None:
            return [], frame_event
        k = np.array(info.k).reshape(3, 3)
        to_cam = tf.lookup(image.header.frame_id, "base_link")

        def project(xyz, to):  # Nx3 -> Nx2 pixels (NaN behind the camera)
            if to is None or not len(xyz):
                return np.zeros((0, 2))
            c = (to @ np.c_[xyz, np.ones(len(xyz))].T).T[:, :3]
            uv = (k @ c.T).T
            uv = uv[:, :2] / np.where(c[:, 2:3] > 0.1, uv[:, 2:3], np.nan)
            return uv

        def inside(uv, bb, margin):
            mx, my = margin * (bb[2] - bb[0]), margin * (bb[3] - bb[1])
            return (uv[:, 0] >= bb[0] - mx) & (uv[:, 0] <= bb[2] + mx) & (uv[:, 1] >= bb[1] - my) & (uv[:, 1] <= bb[3] + my)

        from sensor_msgs_py import point_cloud2

        def xyz_of(cloud):
            p = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
            return np.c_[p["x"], p["y"], p["z"]].astype(np.float64)

        # each box's points: base_link (transformed by fusion; the header frame_id is the camera's)
        obj_uv = [(o.label, project(xyz_of(o.cloud), to_cam)) for o in (objects.objects if objects else [])]
        sel_uv = project(xyz_of(selected), tf.lookup(image.header.frame_id, selected.header.frame_id or "base_link")) \
            if selected is not None else np.zeros((0, 2))
        obstacles = [(o.label, project(np.array([[o.local_point.point.x, o.local_point.point.y, o.local_point.point.z]]),
                                       to_cam)[0], o) for o in det.obstacles]
        events, used_obs, used_obj = [], set(), set()
        for b in boxes.boxes:
            bb = [b.min_x, b.min_y, b.max_x, b.max_y]
            e = {"camera": camera, "image_stamp": stamp, "label": int(b.label), "bbox": bb, "source": "rosbag"}
            obs = next((j for j, (lab, uv, _) in enumerate(obstacles)
                        if j not in used_obs and lab == b.label and inside(uv[None], bb, 0.15)[0]), None)
            obj = max((j for j, (lab, uv) in enumerate(obj_uv) if j not in used_obj and lab == b.label and len(uv)),
                      key=lambda j: inside(obj_uv[j][1], bb, 0.1).mean(), default=None)
            if obj is not None and inside(obj_uv[obj][1], bb, 0.1).mean() < 0.5:
                obj = None
            pts = []
            if obj is not None:
                used_obj.add(obj)
                pts = [[round(u, 1), round(v, 1), -1] for u, v in obj_uv[obj][1] if np.isfinite(u)]
            if obs is not None:
                used_obs.add(obs)
                pts += [[round(u, 1), round(v, 1), 0] for u, v in sel_uv[inside(sel_uv, bb, 0.1)] if np.isfinite(u)]
                o = obstacles[obs][2]
                e.update({"event": "box_detected", "selected_cluster": 0,
                          "position": [round(o.local_point.point.x, 2), round(o.local_point.point.y, 2)]})
            elif obj is None:
                e.update({"event": "box_rejected", "reason": "no_points"})
            else:
                e.update({"event": "box_rejected", "reason": "not_detected_bag"})
            e["points"] = pts[:: max(1, len(pts) // 300)]
            events.append(e)
        return events, frame_event

    def _static_tf(self, source):
        """All /tf_static transforms in the source (several static publishers -> several latched messages)."""
        key = ("tf_static", source, tuple(self.files(source)))
        if key not in self._cache:
            tf = TfGraph()
            for _, schema, _, data in self._iter(source, ["/tf_static"]):
                tf.add(deserialize_message(data, get_message(schema)))
            self._cache[key] = tf
        return self._cache[key]

    def frame(self, source, camera, index):
        cam = self.cameras[camera]
        frames = self.frames(source, camera)
        log_t, stamp = frames[index]
        image = boxes = None
        events, cloud_msg, tf = [], None, TfGraph()
        frame_event = None
        # image and boxes arrive together; fusion log entries up to a few seconds later (log time = laptop receive)
        # (/run_log is published in batches: its entries arrive up to several seconds after the image)
        kfs = keyframe_topics(cam)
        for topic, schema, lt, data in self._iter(source, kfs + [cam["boxes"], "/run_log", "/tf"],
                                                  log_t - 1.0, log_t + 6.0):
            if topic in kfs and abs(header_stamp_ns(data) / 1e9 - stamp) < 1e-6:
                image = deserialize_message(data, get_message(schema))
            elif topic == cam["boxes"] and abs(header_stamp_ns(data) / 1e9 - stamp) < 1e-6:
                boxes = deserialize_message(data, get_message(schema))
            elif topic == "/run_log":
                for line in decode_string_cdr(data).splitlines():
                    if '"image_stamp"' not in line:
                        continue
                    try:
                        e = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if e.get("camera") == log_camera(self.cameras, camera) and abs(e.get("image_stamp", -1) - stamp) < 1e-6:
                        if e["event"] == "frame":
                            frame_event = e
                        else:
                            events.append(e)
            elif topic == "/tf":
                tf.add(deserialize_message(data, get_message(schema)))
        if frame_event is None and not events and self.cameras[camera].get("detections"):
            out = self._fusion_output_for(source, camera, stamp, log_t)  # no run log: fusion's recorded outputs
            if out is not None:
                tf.edges.update(self._static_tf(source).edges)
                info0 = self._latest(source, cam["camera_info"], "sensor_msgs/msg/CameraInfo")
                events, frame_event = self._bag_fusion_events(source, camera, stamp, log_t, out, boxes, image, info0, tf)
        if frame_event and "cloud_stamp" in frame_event:  # the fused scan: written when its frame event arrived
            for topic, schema, lt, data in self._iter(source, [self.cfg.get("keyframe_cloud_topic")],
                                                      log_t - 1.0, log_t + 6.0):
                if abs(header_stamp_ns(data) / 1e9 - frame_event["cloud_stamp"]) < 1e-6:
                    cloud_msg = deserialize_message(data, get_message(schema))
                    break
        clouds = [cloud_msg] if cloud_msg is not None else self._nearest_clouds(source, stamp, log_t)
        tf.edges.update(self._static_tf(source).edges)
        info = self._latest(source, cam["camera_info"], "sensor_msgs/msg/CameraInfo")
        boxes_from_log = boxes is None and any("bbox" in e for e in events)
        if boxes_from_log:  # YOLO's box message wasn't recorded (dropped by the bridge): fusion's logged boxes
            from types import SimpleNamespace
            boxes = SimpleNamespace(boxes=[SimpleNamespace(min_x=e["bbox"][0], min_y=e["bbox"][1], max_x=e["bbox"][2],
                                                           max_y=e["bbox"][3], label=e["label"], probability=None)
                                           for e in events if "bbox" in e and "label" in e])
        out = build_frame(camera, image, boxes, events, frame_event, clouds, info, tf,
                          cam.get("camera_convention", "optical"))
        out.update({"index": index, "count": len(frames), "time": log_t, "stamp": stamp})
        if boxes_from_log:
            out["lidar_note"] = "; ".join(n for n in ("boxes from fusion's log (YOLO message not recorded)",
                                                     out.get("lidar_note")) if n)
        if cloud_msg is None and clouds:
            out["lidar_note"] = "; ".join(n for n in ("no fusion log: nearest scan shown", out.get("lidar_note")) if n)
        return out

    def _nearest_clouds(self, source, stamp, log_t, window=0.15, burst=0.06):
        """Without a fusion `frame` event (e.g. an imported rosbag): the scan nearest to the image
        stamp on the first `cloud_topics` topic that has one, plus the scans within `burst` of it
        (LiDAR drivers that split a revolution into several messages)."""
        for topic in self.cfg.get("cloud_topics", [self.cfg.get("keyframe_cloud_topic")]):
            found = []
            for _, schema, _, data in self._iter(source, [topic], log_t - 1.0, log_t + 1.0):
                dt = header_stamp_ns(data) / 1e9 - stamp
                if abs(dt) <= window:
                    found.append((dt, schema, data))
            if found:
                best = min(found, key=lambda f: abs(f[0]))
                return [deserialize_message(d, get_message(sc)) for dt, sc, d in found if abs(dt - best[0]) <= burst]
        return []


def build_frame(camera, image, boxes, events, frame_event, clouds, info, tf, convention="optical"):
    """One keyframe for the viewer (recorded or live): base64 JPEG, YOLO boxes with their fusion
    outcomes, the fused scan projected into the image + top view, and the frame's log entries."""
    out = {"camera": camera,
           "image": base64.b64encode(bytes(image.data)).decode() if image else None,
           "boxes": _boxes_with_outcomes(boxes, events), "frame_event": frame_event,
           "events": [{k: v for k, v in e.items() if k != "_rec_time"} for e in events]}
    out.update(_lidar([c for c in clouds if c is not None], info, image, tf, frame_event, convention))
    return out


def _boxes_with_outcomes(boxes, events):
    """YOLO boxes, each matched to its fusion event (same label, nearest box center: logged boxes may be padded)."""
    if boxes is None:
        return []
    evs = [e for e in events if e["event"] in ("box_detected", "box_rejected", "box_skipped")]
    used, out = set(), []
    for b in boxes.boxes:
        cx, cy = (b.min_x + b.max_x) / 2, (b.min_y + b.max_y) / 2
        best, best_d = None, 1e9
        for i, e in enumerate(evs):
            if i in used or e.get("label") != b.label or "bbox" not in e:
                continue
            ex, ey = (e["bbox"][0] + e["bbox"][2]) / 2, (e["bbox"][1] + e["bbox"][3]) / 2
            d = abs(ex - cx) + abs(ey - cy)
            if d < best_d:
                best, best_d = i, d
        e = evs[best] if best is not None and best_d < 60 else None
        if e is not None:
            used.add(best)
        outcome = ("detected" if e["event"] == "box_detected" else
                   "skipped" if e["event"] == "box_skipped" else "rejected") if e else "unknown"
        out.append({"bbox": [b.min_x, b.min_y, b.max_x, b.max_y], "label": b.label, "probability": b.probability,
                    "outcome": outcome, "reason": e.get("reason") if e else None,
                    "color": e.get("color") if e else None,
                    "details": {k: v for k, v in e.items() if k not in ("t", "wall", "node", "level", "event", "camera",
                                                                       "image_stamp", "_rec_time")} if e else {}})
    return out



def _lidar(clouds, info, image, tf, frame_event, convention="optical"):
    """Projected points (u, v, range) in the image + top-down points (x, y) in base_link, from one
    scan (or the messages of one LiDAR revolution, same frame)."""
    if not clouds:
        return {"points": [], "bev": [], "lidar_note": "no fused scan recorded for this frame"}
    from sensor_msgs_py import point_cloud2
    cloud = clouds[0]
    parts = []
    for c in clouds:  # structured read: works with mixed field types (e.g. raw Velodyne ring/time)
        p = point_cloud2.read_points(c, field_names=["x", "y", "z"], skip_nans=True)
        parts.append(np.c_[p["x"], p["y"], p["z"]].astype(np.float64))
    xyz = np.concatenate(parts)
    res = {"cloud_points": int(len(xyz)), "cloud_frame": cloud.header.frame_id}
    if frame_event and "image_stamp" in frame_event and "cloud_stamp" in frame_event:
        res["scan_image_dt_ms"] = round((frame_event["cloud_stamp"] - frame_event["image_stamp"]) * 1e3, 1)
    elif image is not None:
        dts = [(c.header.stamp.sec + c.header.stamp.nanosec * 1e-9) -
               (image.header.stamp.sec + image.header.stamp.nanosec * 1e-9) for c in clouds]
        res["scan_image_dt_ms"] = round(min(dts, key=abs) * 1e3, 1)
    homog = np.c_[xyz, np.ones(len(xyz))]
    to_base = tf.lookup("base_link", cloud.header.frame_id)
    if to_base is not None:
        bl = (to_base @ homog.T).T[:, :3]
        keep = np.linalg.norm(bl[:, :2], axis=1) < 60
        sub = bl[keep]  # every point within 60 m
        res["bev"] = np.round(sub[:, :2], 2).tolist()
    else:
        res["bev"] = []
    if info is None or image is None:
        res.update({"points": [], "lidar_note": "no camera_info / image"})
        return res
    to_cam = tf.lookup(image.header.frame_id, cloud.header.frame_id)
    if to_cam is None:
        res.update({"points": [], "lidar_note": f"no TF {cloud.header.frame_id} -> {image.header.frame_id}"})
        return res
    cam = (to_cam @ homog.T).T[:, :3]
    if convention == "gazebo":  # sim camera frame: x forward, y left, z up -> optical (as fusion's is_sim)
        cam = np.c_[-cam[:, 1], -cam[:, 2], cam[:, 0]]
    cam = cam[cam[:, 2] > 0.1]
    k = np.array(info.k).reshape(3, 3)
    uv = (k @ cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    rng = np.linalg.norm(cam, axis=1)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < info.width) & (uv[:, 1] >= 0) & (uv[:, 1] < info.height)
    pts = np.c_[uv[inside], rng[inside]]
    res["points"] = np.round(pts, 1).tolist()
    res["image_size"] = [info.width, info.height]
    return res
