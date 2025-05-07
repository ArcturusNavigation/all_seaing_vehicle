#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="localization_$current_date_time"
echo "Recording localization to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /mavros/global_position/global /mavros/imu/data /odometry/filtered /odometry/gps /tf /tf_static