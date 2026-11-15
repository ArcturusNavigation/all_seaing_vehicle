#!/usr/bin/env python3
"""
Offline conversion of a ROS 2 bag that adds raw sensor_msgs/Image topics decoded from its
sensor_msgs/CompressedImage topics.

Purpose: replay bags on the Jetson with the same input topics the boat has live, without
running a decompression node during the replay. That node would add CPU load the real
boat doesn't have, and make performance measurements unrepresentative.

Topic naming: `<base>/compressed` becomes `<base>`. For example,
`/back_left_oak/rgb/image_rect/compressed` becomes `/back_left_oak/rgb/image_rect`, which is
what perception.launch.py subscribes to. Every other topic is copied unchanged, with the same
timestamps and the same recorded QoS profiles.

Usage:
    ros2 run all_seaing_utility rosbag_decompress_images.py -i <input_bag_dir> -o <output_bag_dir>
        [-t <compressed_topic> ...]   # default: every sensor_msgs/msg/CompressedImage topic
        [--drop-compressed]           # don't copy the original compressed topics
        [--encoding bgr8]             # output Image encoding (default bgr8)

The output is a new sqlite3 bag directory; the input bag is never modified.
"""

from argparse import ArgumentParser
import sys

from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CompressedImage

COMPRESSED_TYPE = "sensor_msgs/msg/CompressedImage"
RAW_TYPE = "sensor_msgs/msg/Image"
COMPRESSED_SUFFIX = "/compressed"


def raw_topic_name(compressed_topic):
    """Name of the raw topic for a compressed topic (strip a trailing '/compressed')."""
    if compressed_topic.endswith(COMPRESSED_SUFFIX):
        return compressed_topic[: -len(COMPRESSED_SUFFIX)]
    return compressed_topic + "/raw"


def main():
    parser = ArgumentParser(description="add raw Image topics decoded from CompressedImage topics of a ROS 2 bag")
    parser.add_argument("-i", "--input", help="input ROS 2 bag directory", type=str, required=True)
    parser.add_argument("-o", "--output", help="output ROS 2 bag directory (must not exist)", type=str, required=True)
    parser.add_argument("-t", "--topics", help="compressed topics to decode (default: all CompressedImage topics)",
                        type=str, nargs="+", default=None)
    parser.add_argument("--drop-compressed", help="don't copy the original compressed topics", action="store_true")
    parser.add_argument("--encoding", help="encoding of the output Image messages", type=str, default="bgr8")
    args = parser.parse_args()

    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.input, storage_id="sqlite3"), converter_options)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3"), converter_options)

    # Decide which compressed topics to decode
    all_topics = {t.name: t for t in reader.get_all_topics_and_types()}
    if args.topics is None:
        decode = {name for name, t in all_topics.items() if t.type == COMPRESSED_TYPE}
    else:
        decode = set(args.topics)
        for name in decode:
            if name not in all_topics or all_topics[name].type != COMPRESSED_TYPE:
                sys.exit(f"Topic {name} is not a {COMPRESSED_TYPE} topic in the input bag")
    raw_names = {name: raw_topic_name(name) for name in decode}
    for name, raw in raw_names.items():
        if raw in all_topics:
            sys.exit(f"Raw topic {raw} already exists in the input bag, refusing to overwrite it")

    # Register output topics, keeping the recorded QoS so playback behaves like the original
    for name, t in all_topics.items():
        if not (args.drop_compressed and name in decode):
            writer.create_topic(t)
        if name in decode:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=raw_names[name], type=RAW_TYPE,
                serialization_format=t.serialization_format,
                offered_qos_profiles=t.offered_qos_profiles))
    print(f"Decoding {len(decode)} topic(s): " + ", ".join(f"{n} -> {raw_names[n]}" for n in sorted(decode)))

    bridge = CvBridge()
    decoded = 0
    copied = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in decode:
            compressed = deserialize_message(data, CompressedImage)
            raw = bridge.cv2_to_imgmsg(bridge.compressed_imgmsg_to_cv2(compressed, args.encoding), args.encoding)
            raw.header = compressed.header  # keep stamp & frame_id for TF lookups and synchronizers
            writer.write(raw_names[topic], serialize_message(raw), t)
            decoded += 1
            if args.drop_compressed:
                continue
        writer.write(topic, data, t)
        copied += 1

    del writer  # flushes and writes metadata.yaml
    print(f"Done: {decoded} image(s) decoded, {copied} message(s) copied -> {args.output}")


if __name__ == "__main__":
    main()
