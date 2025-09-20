#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="follow_path_$current_date_time"
echo "Recording follow the path to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /waypoint_markers /thrusters/front_right/thrust /thrusters/front_left/thrust /thrusters/back_left/thrust /thrusters/back_right/thrust /control_options /cmd_vel /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /point_cloud/filtered /detections/front /detections/merged /detections/back_left /detections/back_right /mavros/global_position/global /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag /mavros/local_position/odom /mavros/global_position/raw/fix /odometry/filtered /odometry/gps /tf /tf_static /obstacle_map/local /obstacle_map/global /obstacle_map/raw /obstacle_map/unlabeled /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /back_left_oak/rgb/image_rect/compressed /back_left_oak/rgb/camera_info /back_right_oak/rgb/image_rect/compressed /back_right_oak/rgb/camera_info