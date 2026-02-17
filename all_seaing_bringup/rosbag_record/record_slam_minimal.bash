#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="slam_$current_date_time"
echo "Recording lidar & camera & detections & localization & maps to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o /media/arcturus/drive/rosbags/$file_name /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /point_cloud/filtered /detections/front /detections/merged /detections/back_left /detections/back_right /labeled_object_point_clouds/front /labeled_object_point_clouds/back_left /labeled_object_point_clouds/back_right /labeled_object_point_clouds/merged /mavros/global_position/global /mavros/imu/data /mavros/imu/data_raw /mavros/imu/mag /mavros/local_position/odom /mavros/global_position/raw/fix /odometry/filtered /odometry/gps odom_rf2o/filtered /tf /tf_static /obstacle_map/local /obstacle_map/global /obstacle_map/raw /obstacle_map/unlabeled /object_planes /object_planes/global /object_planes_viz /obstacle_map/map_cov_viz