#!/usr/bin/env python3

from argparse import ArgumentParser
import subprocess
from tempfile import TemporaryDirectory
from typing import List

import cv2
import cv_bridge
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image


def ffmpeg(args: List[str]):
    return subprocess.run(["ffmpeg"] + args, check=True)


def convert(input_path: str, input_topic: str, output_path: str):

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=input_path, storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    bridge = cv_bridge.CvBridge()
    tmp_dir = TemporaryDirectory()
    i = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if input_topic == topic:
            i += 1
            msg = deserialize_message(data, Image)
            image = bridge.imgmsg_to_cv2(msg, "bgr8")
            path = tmp_dir.name + "/" + str(i).zfill(5) + ".png"
            cv2.imwrite(path, image)

    ffmpeg(
        [
            "-framerate",
            "30",
            "-pattern_type",
            "glob",
            "-i",
            tmp_dir.name + "/*.png",
            "-c:v",
            "libx264",
            output_path,
            "-y",
            "-loglevel",
            "panic",
        ]
    )

    tmp_dir.cleanup()


def main():
    parser = ArgumentParser(
        prog="rosbag_to_mp4.py",
        description="converts a rosbag image topic to an mp4",
    )
    parser.add_argument(
        "-i", "--input", help="input rosbag path", type=str, required=True
    )
    parser.add_argument(
        "-t", "--topic", help="image topic to convert", type=str, required=True
    )
    parser.add_argument(
        "-o", "--output", help="output mp4 path", type=str, required=True
    )
    args = parser.parse_args()

    convert(input_path=args.input, topic=args.topic, output_path=args.output)


if __name__ == "__main__":
    main()
