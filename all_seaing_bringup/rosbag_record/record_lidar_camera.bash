#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="lidar_camera_$current_date_time"
echo "Recording lidar & camera & detections to $file_name"
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /bounding_boxes /shape_boxes /static_shape_boxes /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy /velodyne_points /point_cloud/filtered /obstacle_map/raw /obstacle_map/labeled /obstacle_map/refined_untracked /labeled_object_point_clouds /refined_object_point_clouds_segments