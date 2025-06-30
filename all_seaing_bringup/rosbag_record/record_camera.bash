#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="camera_$current_date_time"
echo "Recording camera & detections to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /back_left_oak/rgb/image_rect /back_left_oak/rgb/camera_info /back_right_oak/rgb/image_rect /back_right_oak/rgb/camera_info /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy /annotated_image/buoy/back_left /annotated_image/buoy/back_right