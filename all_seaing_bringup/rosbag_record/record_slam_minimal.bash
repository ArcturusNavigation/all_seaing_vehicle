#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="slam_$current_date_time"
echo "Recording lidar & camera & detections & localization & maps to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /velodyne_points /point_cloud/filtered /obstacle_map/raw /obstacle_map/labeled /labeled_object_point_clouds /refined_object_point_clouds_segments /refined_object_point_clouds_segments/merged /refined_object_point_clouds_segments/back_left /refined_object_point_clouds_segments/back_right /mavros/global_position/global /mavros/imu/data /odometry/filtered /odometry/gps /tf /tf_static /obstacle_map/refined_untracked /obstacle_map/refined_tracked