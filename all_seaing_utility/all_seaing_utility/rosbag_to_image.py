#!/usr/bin/env python3

# python3 /home/arcturus/arcturus/dev_ws/src/all_seaing_vehicle/all_seaing_utility/all_seaing_utility/rosbag_to_image.py /home/arcturus/arcturus/test_05-21-24/test1/test1_0.db3 /zed/zed_node/rgb/image_rect_color /home/arcturus/arcturus/bag_output

from argparse import ArgumentParser
import cv2
from cv_bridge import CvBridge
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
import rclpy

def main():
    rclpy.init()

    # so you can input info everytime, totally could hardcode tho
    parser = ArgumentParser(description="Extract images from ROS2 bag")
    parser.add_argument("bag_file", help="Input ROS2 bag file")
    parser.add_argument("image_topic", help="Image topic to convert")
    parser.add_argument("output_dir", help="Output directory")

    args = parser.parse_args()

    bag_file = args.bag_file
    image_topic = args.image_topic
    output_dir = args.output_dir

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    count = 0

    print(f"Extracting images from {bag_file} on topic {image_topic} into {output_dir}")

    choose_framerate = 5
    current_rate = 579//38.314721344 #15
    count1 = 0

    framerate = max(1, current_rate//choose_framerate)

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == image_topic and count1%framerate == 0:
            msg = deserialize_message(data, Image)

            # ROS2 Image to format that works with OpenCV
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            image_path = os.path.join(output_dir, f"frame{count:06}.png")
            cv2.imwrite(image_path, cv_img)
            print(f"Wrote image {count} to {image_path}")

            count += 1
        count1 += 1

    print(f"Extracted {count} images.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
