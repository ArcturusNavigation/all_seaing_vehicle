#!/usr/bin/env python3
"""
Shift the recording timestamps of a ROS 2 bag (the times `ros2 bag play --clock` publishes
on /clock) by a constant offset, so they match the clock the messages were stamped with.

Why: when a bag is re-recorded (e.g. played on a laptop and recorded again), the message
header stamps keep the original clock (e.g. the Jetson's clock) while the recording
timestamps use the re-recording machine's clock. On replay with --clock, nodes then see a
`now()` unrelated to the stamps, which breaks message-age measurements and anything that
compares stamps with the current time (TF/odometry lookups at message time).

Offset: by default the median of (header stamp - recording time) over messages whose type
starts with a std_msgs/Header (sampled), or a fixed `--offset` in seconds. Message contents
are not modified. The output is a new sqlite3 bag; the input is never modified.

Usage:
    ros2 run all_seaing_utility rosbag_shift_time.py -i <input_bag> -o <output_bag> [--offset SECONDS]
"""

from argparse import ArgumentParser
import statistics
import struct

import rosbag2_py

# Message types whose first field is a std_msgs/Header (stamp right after the CDR encapsulation header)
HEADER_FIRST_TYPES = ("sensor_msgs/msg/", "nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped",
                      "geometry_msgs/msg/TwistStamped", "all_seaing_interfaces/msg/LabeledBoundingBox2DArray")


def header_stamp(data):
    """Header stamp [ns] of a CDR little-endian serialized message starting with a std_msgs/Header."""
    if data[1] != 0x01:  # encapsulation kind: 0x0001 = CDR_LE
        return None
    sec, nanosec = struct.unpack_from("<iI", data, 4)
    return sec * 1_000_000_000 + nanosec


def estimate_offset(uri, converter_options, max_samples=5000):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"), converter_options)
    topics = [t.name for t in reader.get_all_topics_and_types() if t.type.startswith(HEADER_FIRST_TYPES)]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    diffs = []
    while reader.has_next() and len(diffs) < max_samples:
        _, data, t = reader.read_next()
        stamp = header_stamp(data)
        if stamp:
            diffs.append(stamp - t)
    if not diffs:
        raise SystemExit("No header-stamped messages found, pass --offset explicitly")
    return int(statistics.median(diffs)), len(diffs)


def main():
    parser = ArgumentParser(description="shift the recording timestamps of a ROS 2 bag by a constant offset")
    parser.add_argument("-i", "--input", help="input ROS 2 bag directory", type=str, required=True)
    parser.add_argument("-o", "--output", help="output ROS 2 bag directory (must not exist)", type=str, required=True)
    parser.add_argument("--offset", help="offset to add [s] (default: estimated from header stamps)", type=float)
    args = parser.parse_args()

    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    if args.offset is None:
        offset_ns, n = estimate_offset(args.input, converter_options)
        print(f"Estimated offset {offset_ns / 1e9:.3f} s (median of header stamp - recording time over {n} messages)")
    else:
        offset_ns = int(args.offset * 1e9)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.input, storage_id="sqlite3"), converter_options)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3"), converter_options)
    for topic in reader.get_all_topics_and_types():
        writer.create_topic(topic)

    count = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        writer.write(topic, data, t + offset_ns)
        count += 1
    del writer  # flushes and writes metadata.yaml
    print(f"Done: {count} message(s) shifted by {offset_ns / 1e9:.3f} s -> {args.output}")


if __name__ == "__main__":
    main()
