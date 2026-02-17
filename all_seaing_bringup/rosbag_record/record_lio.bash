#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="localization_$current_date_time"
echo "Recording localization to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o /media/arcturus/drive/rosbags/$file_name /mavros/global_position/global /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag /mavros/local_position/local /mavros/local_position/pose /mavros/local_position/odom /mavros/global_position/raw/fix /odometry/filtered /odometry/gps /tf /tf_static /velodyne_points