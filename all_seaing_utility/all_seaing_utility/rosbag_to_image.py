#!/usr/bin/env python3

from argparse import ArgumentParser
import cv2
from cv_bridge import CvBridge
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

def main():
    parser = ArgumentParser(description="extract images from ROS2 bag")
    parser.add_argument("-i", "--input", help="input ROS2 bag file", type=str, required=True)
    parser.add_argument("-t", "--topic", help="image topic to convert", type=str, required=True)
    parser.add_argument("-o", "--output", help="output directory", type=str, required=True)
    parser.add_argument("--fps", help="frames per second", type=int, default=30)

    args = parser.parse_args()
    bag_file = args.input
    image_topic = args.topic
    output_dir = args.output
    choose_framerate = args.fps

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    count = 0

    msg_count = 0
    start_time = None
    end_time = None
    current_rate = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
    
        if topic == image_topic:
            msg = deserialize_message(data, Image)
            if msg_count == 0:
                start_time = t
            msg_count += 1
            end_time = t

    if msg_count > 1:
        duration = (end_time - start_time) / 1e9  # Convert to seconds
        current_rate = msg_count / duration
        print("Current rate:", current_rate)
    else:
        print("Not enough messages in the bag.")

    print(f"Extracting images from {bag_file} on topic {image_topic} into {output_dir}")

    count1 = 0
    framerate = max(1, current_rate//choose_framerate)
    print("framerate :: ", framerate)

    reader.open(storage_options, converter_options)

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == image_topic and count1 % framerate == 0:
            msg = deserialize_message(data, Image)

            # ROS2 Image to format that works with OpenCV
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            image_path = os.path.join(output_dir, f"frame{count:06}.png")
            cv2.imwrite(image_path, cv_img)
            print(f"Wrote image {count} to {image_path}")

            count += 1
        count1 += 1

    print(f"Extracted {count} images.")

if __name__ == '__main__':
    main()
