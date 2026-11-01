#!/usr/bin/env python3
"""
Light copy of a ROS 2 bag, for viewing it on a laptop (control panel Recordings tab) or sharing it:
the heavy topics are reduced, everything else is copied unchanged (same timestamps and QoS).

- sensor_msgs/Image           -> JPEG sensor_msgs/CompressedImage on `<topic>/keyframes/compressed`,
                                 at most --image-rate per topic (header kept, so TF/boxes still match);
                                 images that YOLO boxes were published for (a LabeledBoundingBox2DArray
                                 with the same header stamp) are preferred, others only fill gaps
                                 longer than 2 periods
- sensor_msgs/CompressedImage -> re-encoded as JPEG (same quality) on `<base>/keyframes/compressed`
                                 (`<base>/compressed` renamed), same rate cap
- sensor_msgs/PointCloud2     -> kept at most --cloud-rate per topic, but a scan published within
                                 --cloud-burst of the previous kept one is kept too, so LiDAR
                                 drivers that split one revolution into several messages keep
                                 whole revolutions; scans that LiDAR-camera fusion used (the stamps
                                 of its /detections/* outputs, which carry the scan's stamp) are
                                 preferred, others only fill gaps longer than 2 periods
- --exclude topics are dropped.

The output is a sqlite3 bag directory (playable with `ros2 bag play`) plus `schemas.json` in it:
{type: ros2msg definition text (with its dependencies)}, since sqlite3 bags don't store message
definitions and MCAP readers (Foxglove, the control panel) need them.

Progress for callers: lines `PROGRESS <fraction 0..1>` on stdout, and `DONE <output dir>` at the end.

Usage:
    ros2 run all_seaing_utility rosbag_light_copy.py -i <bag_dir> -o <output_dir>
        [--image-rate 5] [--cloud-rate 5] [--cloud-burst 0.06] [--jpeg-quality 80] [--exclude <topic> ...]
"""

from argparse import ArgumentParser
import json
import os
import re
import sys

import cv2
import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image

BOXES = "all_seaing_interfaces/msg/LabeledBoundingBox2DArray"
IMAGE = "sensor_msgs/msg/Image"
COMPRESSED = "sensor_msgs/msg/CompressedImage"
CLOUD = "sensor_msgs/msg/PointCloud2"
PRIMITIVES = {"bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32",
              "uint32", "int64", "uint64", "string", "wstring"}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


# ------------------------------------------------------------------ message definitions

def _msg_text(pkg, name):
    with open(os.path.join(get_package_share_directory(pkg), "msg", name + ".msg")) as f:
        return f.read()


def msg_definition(type_name):
    """ros2msg definition of 'pkg/msg/Type' with its dependencies (MCAP 'ros2msg' schema encoding)."""
    pkg, _, name = type_name.split("/")
    main = _msg_text(pkg, name)
    deps, order, todo = {}, [], [(pkg, main)]
    while todo:
        cur_pkg, text = todo.pop()
        for line in text.splitlines():
            words = line.split("#", 1)[0].split()
            if len(words) < 2 or "=" in words[1]:  # blank, comment or constant
                continue
            ftype = re.sub(r"<=\d+$", "", re.sub(r"\[.*?\]$", "", words[0]))  # strip array / string bounds
            if ftype in PRIMITIVES:
                continue
            parts = ftype.split("/")  # pkg/Type, pkg/msg/Type, or Type (same package)
            dpkg, dname = (parts[0], parts[-1]) if len(parts) > 1 else (cur_pkg, parts[0])
            key = f"{dpkg}/{dname}"
            if key not in deps:
                deps[key] = _msg_text(dpkg, dname)
                order.append(key)
                todo.append((dpkg, deps[key]))
    sep = "\n" + "=" * 80 + "\n"
    return main + "".join(f"{sep}MSG: {k}\n{deps[k]}" for k in order)


# ------------------------------------------------------------------ conversion

def to_jpeg(img, quality):
    frame = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1)
    if img.encoding in ("rgb8", "rgba8"):
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR if img.encoding == "rgb8" else cv2.COLOR_RGBA2BGR)
    elif frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return CompressedImage(header=img.header, format="jpeg", data=jpg.tobytes()) if ok else None


def recompress(msg, quality):
    """CompressedImage (any format OpenCV decodes, e.g. high-quality JPEG or PNG) as JPEG at `quality`."""
    frame = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return msg
    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return CompressedImage(header=msg.header, format="jpeg", data=jpg.tobytes()) if ok else msg


def keyframe_topic(topic, ttype):
    base = topic[: -len("/compressed")] if ttype == COMPRESSED and topic.endswith("/compressed") else topic
    return base.rstrip("/") + "/keyframes/compressed"


def main():
    parser = ArgumentParser(description="light copy of a ROS 2 bag (JPEG images and capped image/LiDAR rates)")
    parser.add_argument("-i", "--input", required=True, help="input bag directory")
    parser.add_argument("-o", "--output", required=True, help="output bag directory (must not exist)")
    parser.add_argument("--image-rate", type=float, default=5.0, help="max images per second per camera topic")
    parser.add_argument("--cloud-rate", type=float, default=5.0, help="max point clouds per second per topic")
    parser.add_argument("--cloud-burst", type=float, default=0.06,
                        help="also keep a cloud this close [s] after a kept one (split LiDAR revolutions)")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--exclude", nargs="*", default=[], help="topics to drop")
    args = parser.parse_args()
    if os.path.exists(args.output):
        sys.exit(f"{args.output} already exists")

    conv = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.input, storage_id="sqlite3"), conv)
    total = max(1, reader.get_metadata().message_count)
    topics = {t.name: t for t in reader.get_all_topics_and_types() if t.name not in args.exclude}
    # first pass over light topics: stamps of images with YOLO boxes, and of the scans fusion used
    boxed, fused = set(), set()
    box_topics = [n for n, t in topics.items() if t.type == BOXES]
    fusion_topics = [n for n in topics if n.startswith("/detections/")]
    if box_topics or fusion_topics:
        first = rosbag2_py.SequentialReader()
        first.open(rosbag2_py.StorageOptions(uri=args.input, storage_id="sqlite3"), conv)
        first.set_filter(rosbag2_py.StorageFilter(topics=box_topics + fusion_topics))
        while first.has_next():
            topic, d, _ = first.read_next()
            if topic in box_topics:
                boxed.add(int.from_bytes(d[4:8], "little", signed=True) * 1_000_000_000 + int.from_bytes(d[8:12], "little"))
            else:  # ObstacleMap: scan stamp in header, or only in local_header (older bbox_project_pcloud)
                m = deserialize_message(d, get_message(topics[topic].type))
                st = m.header.stamp if m.header.stamp.sec else m.local_header.stamp
                fused.add(stamp_ns(st))
        del first
        print(f"{len(boxed)} box messages, {len(fused)} fusion outputs: their images/scans are preferred", flush=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3"), conv)

    out_name, schemas = {}, {}
    for name, t in topics.items():
        out_type = COMPRESSED if t.type in (IMAGE, COMPRESSED) else t.type
        out_name[name] = keyframe_topic(name, t.type) if t.type in (IMAGE, COMPRESSED) else name
        writer.create_topic(rosbag2_py.TopicMetadata(name=out_name[name], type=out_type,
                                                     serialization_format=t.serialization_format,
                                                     offered_qos_profiles=t.offered_qos_profiles))
        if out_type not in schemas:
            try:
                schemas[out_type] = msg_definition(out_type)
            except (OSError, LookupError, ValueError) as e:  # type not installed here: name-only schema
                print(f"no definition for {out_type}: {e}", flush=True)
                schemas[out_type] = ""

    last_kept = {}  # topic -> header stamp ns of the last kept image/cloud
    n = written = 0
    step = max(1, total // 200)
    while reader.has_next():
        topic, data, t = reader.read_next()
        n += 1
        if n % step == 0:
            print(f"PROGRESS {n / total:.4f}", flush=True)
        if topic not in topics:
            continue
        ttype = topics[topic].type
        if ttype in (IMAGE, COMPRESSED, CLOUD):
            # header stamp right after the CDR encapsulation header (all three start with a Header)
            s = int.from_bytes(data[4:8], "little", signed=True) * 1_000_000_000 + int.from_bytes(data[8:12], "little")
            prev = last_kept.get(topic)
            period = 1e9 / (args.cloud_rate if ttype == CLOUD else args.image_rate)
            preferred = fused if ttype == CLOUD else boxed
            if preferred and s not in preferred:
                period *= 2  # others only fill gaps
            if prev is None or s < prev or s - prev >= period:
                last_kept[topic] = s  # kept: starts a new group
            elif not (ttype == CLOUD and s - prev <= args.cloud_burst * 1e9):
                continue  # within the rate period and not part of the kept revolution
            if ttype == IMAGE:
                msg = to_jpeg(deserialize_message(data, Image), args.jpeg_quality)
                if msg is None:
                    continue
                data = serialize_message(msg)
            elif ttype == COMPRESSED:
                data = serialize_message(recompress(deserialize_message(data, CompressedImage), args.jpeg_quality))
        writer.write(out_name[topic], data, t)
        written += 1

    del writer  # flushes and writes metadata.yaml
    with open(os.path.join(args.output, "schemas.json"), "w") as f:
        json.dump(schemas, f)
    print("PROGRESS 1.0", flush=True)
    print(f"{written} of {n} messages written", flush=True)
    print(f"DONE {args.output}", flush=True)


if __name__ == "__main__":
    main()
