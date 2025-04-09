#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="camera_$current_date_time"
echo "Recording camera & detections to $file_name"
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /bounding_boxes /shape_boxes /static_shape_boxes /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy