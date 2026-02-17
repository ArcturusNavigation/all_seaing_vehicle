#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="meche_$current_date_time"
echo "Recording meche to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o /media/arcturus/drive/rosbags/$file_name /thrusters/front_right/thrust /thrusters/front_left/thrust /thrusters/back_left/thrust /thrusters/back_right/thrust /control_options /cmd_vel /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag /odometry/gps /tf /tf_static