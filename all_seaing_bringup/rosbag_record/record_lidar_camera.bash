#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="lidar_camera_$current_date_time"
echo "Recording lidar & camera & detections to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /back_left_oak/rgb/image_rect /back_left_oak/rgb/camera_info /back_right_oak/rgb/image_rect /back_right_oak/rgb/camera_info /bounding_boxes /bounding_boxes/back_left /bounding_boxes/back_right /shape_boxes /static_shape_boxes /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy /annotated_image/buoy/back_left /annotated_image/buoy/back_right /velodyne_points /point_cloud/filtered /obstacle_map/raw /obstacle_map/labeled /obstacle_map/refined_untracked /labeled_object_point_clouds /refined_object_point_clouds_segments /refined_object_point_clouds_viz /refined_object_segments_viz /refined_object_point_clouds_segments/back_left /refined_object_point_clouds_viz/back_left /refined_object_segments_viz/back_left /refined_object_point_clouds_segments/back_right /refined_object_point_clouds_viz/back_right /refined_object_segments_viz/back_right /tf /tf_static