#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="lidar_$current_date_time"
echo "Recording lidar to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /velodyne_points /point_cloud/filtered