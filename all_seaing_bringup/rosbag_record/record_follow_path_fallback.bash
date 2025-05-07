#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="follow_path_fallback_$current_date_time"
echo "Recording follow the path fallback approach to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /bounding_boxes /shape_boxes /static_shape_boxes /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy /velodyne_points /point_cloud/filtered /obstacle_map/raw /obstacle_map/labeled /obstacle_map/refined_untracked /labeled_object_point_clouds /refined_object_point_clouds_segments /thrusters/front_right/thrust /thrusters/front_left/thrust /thrusters/back_left/thrust /thrusters/back_right/thrust /mavros/imu/data