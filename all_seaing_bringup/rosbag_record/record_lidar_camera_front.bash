#!/bin/bash

current_date_time=$(date +"%m_%d__%H_%M")
file_name="lidar_camera_$current_date_time"
echo "Recording lidar & camera & detections to $file_name"
# export ROS_DOMAIN_ID=10
ros2 bag record -o ../rosbags/$file_name /zed/zed_node/rgb/image_rect_color /zed/zed_node/rgb/camera_info /bounding_boxes /shape_boxes /static_shape_boxes /shape_boxes/back_left /static_shape_boxes/back_left /shape_boxes/back_right /static_shape_boxes/back_right /image/segmented /bounding_boxes_ycrcb /image/segmented_ycrcb /annotated_image/buoy /velodyne_points /point_cloud/filtered /point_cloud/filtered_downsampled /obstacle_map/raw /obstacle_map/unlabeled /obstacle_map/local /labeled_object_point_clouds/front /detections/front /detections/merged /refined_object_point_clouds_viz/front /refined_object_segments_viz/front /tf /tf_static