#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="slam_$current_date_time"
echo "Recording lidar & camera & detections & localization & maps to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /velodyne_points /point_cloud/filtered /obstacle_map/raw /obstacle_map/labeled /detections/front /detections/merged /detections/back_left /detections/back_right /mavros/global_position/global /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag /mavros/local_position/odom /odometry/filtered /odometry/gps /tf /tf_static /obstacle_map/refined_untracked /obstacle_map/refined_tracked