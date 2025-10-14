#!/usr/bin/env python3

from argparse import ArgumentParser
import cv2
from cv_bridge import CvBridge
import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

def main():
    parser = ArgumentParser(description="extract point cloud from ROS2 bag")
    parser.add_argument("-i", "--input", help="input ROS2 bag file", type=str, required=True)
    parser.add_argument("-t", "--topic", help="point cloud topic to convert", type=str, required=True)
    parser.add_argument("-o", "--output", help="output directory", type=str, required=True)
    parser.add_argument("--fps", help="frames per second", type=int, default=30)

    args = parser.parse_args()
    bag_file = args.input
    pcl_topic = args.topic
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
    
        if topic == pcl_topic:
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

    print(f"Extracting images from {bag_file} on topic {pcl_topic} into {output_dir}")

    count1 = 0
    framerate = max(1, current_rate//choose_framerate)
    print("framerate :: ", framerate)

    reader.open(storage_options, converter_options)

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == pcl_topic and count1 % framerate == 0:
            msg = deserialize_message(data, PointCloud2)

            lidar_point_cloud = pc2.read_points(msg)[['x', 'y', 'z']] # list with shape [num_pts, 3], where second dimension is xyz
            lidar_point_cloud = np.hstack([np.expand_dims(lidar_point_cloud['x'].astype(np.float32), axis=0).T, np.expand_dims(lidar_point_cloud['y'].astype(np.float32), axis=0).T, np.expand_dims(lidar_point_cloud['z'].astype(np.float32), axis=0).T])
            # print(lidar_point_cloud.shape)
            lidar_point_cloud = lidar_point_cloud[~np.isnan(lidar_point_cloud)].reshape((-1, 3))
            # print(lidar_point_cloud)

            pcl_path = os.path.join(output_dir, f"frame{count:06}.npz")
            if not os.path.isdir(output_dir):
                os.mkdir(output_dir)
            np.savez(pcl_path, pcl=lidar_point_cloud)
            print(f"Wrote point cloud {count} to {pcl_path}")

            count += 1
        count1 += 1

    print(f"Extracted {count} point clouds.")

if __name__ == '__main__':
    main()
