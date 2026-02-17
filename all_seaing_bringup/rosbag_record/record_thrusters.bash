#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="thrusters_$current_date_time"
echo "Recording thrusters to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o /media/arcturus/drive/rosbags/$file_name /thrusters/front_right/thrust /thrusters/front_left/thrust /thrusters/back_left/thrust /thrusters/back_right/thrust