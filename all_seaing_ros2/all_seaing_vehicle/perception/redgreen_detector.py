#!/usr/bin/env python3
"""
References:
https://pytorch.org/hub/ultralytics_yolov5/
https://pypi.org/project/yolo5/

List of classes:
https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml
"""

import getpass
import torch
import cv_bridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from ament_index_python.packages import get_package_share_directory


class Yolov5Detector(Node):

    def __init__(self):
        super().__init__("yolov5_detector")

        self.bridge = cv_bridge.CvBridge()


        # Subscribers and publishers
        qos_profile = QoSProfile(depth=1)
        self.bbox_pub = self.create_publisher(
            LabeledBoundingBox2DArray, "/perception_suite/bounding_boxes", qos_profile
        )
        self.img_pub = self.create_publisher(
            Image, "/perception_suite/segmented_image", qos_profile
        )
        self.img_sub = self.create_subscription(
            Image,
            #"/perception_suite/image",  # Remap this to correct topic
            "/zed/zed_node/rgb/image_rect_color",  # Remap this to correct topic
            self.img_callback,
            qos_profile,
        )

    def img_callback(self, img):
        try:
            img = self.bridge.imgmsg_to_cv2(img, "hsv")
        except cv_bridge.CvBridgeError as e:
            self.get_logger().info(str(e))
        
        seg_red_1=cv2.inRange(img, [0,0,0], [15,255,255])
        seg_red_2=cv2.inRange(img, [165,0,0], [180,255,255])
        seg_red=add(seg_red_1,seg_red_2)
	seg_green=cv2.inRange(img, [45,0,0], [75,255,255])
	[row,col]=seg_red.shape()
	for i in range(row):
		for j in range(col):
			if seg_red[i,j] !=0:
				min_x=10000
				max_x=0
				min_y=10000
				max_y=0
				queue=[[i,j]]
				seg_red[i,j]=0
				st_queue=0
				end_queue=1
				while st_queue<end_queue:
					cur=queue[st_queue]
					if cur[0]>max_x:
						max_x=cur[0]
					if cur[0]<min_x:
						min_x=cur[0]
					if cur[1]>max_y:
						max_x=cur[0]
					if cur[1]<min_y:
						min_x=cur[0]	
					if cur[0]>0 and seg_red[cur[0]-1,cur[1]]!=0:
						queue[end_queue]=[cur[0]-1,cur[1]]
						seg_red[cur[0]-1,cur[1]]=0
						end_queue+=1
					if cur[1]>0 and seg_red[cur[0],cur[1]-1]!=0:
						queue[end_queue]=[cur[0],cur[1]-1]
						seg_red[cur[0],cur[1]-1]=0
						end_queue+=1
					if cur[0]+1<row and seg_red[cur[0]+1,cur[1]]!=0:
						queue[end_queue]=[cur[0]+1,cur[1]]
						seg_red[cur[0]+1,cur[1]]=0
						end_queue+=1
					if cur[1]+1<col and seg_red[cur[0],cur[1]+1]!=0:
						queue[end_queue]=[cur[0],cur[1]+1]
						seg_red[cur[0],cur[1]+1]=0
						end_queue+=1
					st_queue+=1
				detected_objects.append([min_x,min_y,max_x,max_y,"red")
			if seg_green[i,j] !=0:
				min_x=10000
				max_x=0
				min_y=10000
				max_y=0
				queue=[[i,j]]
				seg_green[i,j]=0
				st_queue=0
				end_queue=1
				while st_queue<end_queue:
					cur=queue[st_queue]
					if cur[0]>max_x:
						max_x=cur[0]
					if cur[0]<min_x:
						min_x=cur[0]
					if cur[1]>max_y:
						max_x=cur[0]
					if cur[1]<min_y:
						min_x=cur[0]	
					if cur[0]>0 and seg_green[cur[0]-1,cur[1]]!=0:
						queue[end_queue]=[cur[0]-1,cur[1]]
						seg_green[cur[0]-1,cur[1]]=0
						end_queue+=1
					if cur[1]>0 and seg_green[cur[0],cur[1]-1]!=0:
						queue[end_queue]=[cur[0],cur[1]-1]
						seg_green[cur[0],cur[1]-1]=0
						end_queue+=1
					if cur[0]+1<row and seg_green[cur[0]+1,cur[1]]!=0:
						queue[end_queue]=[cur[0]+1,cur[1]]
						seg_green[cur[0]+1,cur[1]]=0
						end_queue+=1
					if cur[1]+1<col and seg_green[cur[0],cur[1]+1]!=0:
						queue[end_queue]=[cur[0],cur[1]+1]
						seg_green[cur[0],cur[1]+1]=0
						end_queue+=1
					st_queue+=1
				detected_objects.append([min_x,min_y,max_x,max_y,"green")
						

        # Set header of bboxes
        bboxes = LabeledBoundingBox2DArray()
        bboxes.header.stamp = self.get_clock().now().to_msg()

        # Set bounding boxes around colored buoys
        for object in detected_objects:
            bbox = LabeledBoundingBox2D()

            bbox.min_x = object[0]
            bbox.min_y = object[1]

            bbox.max_x = object[2]
            bbox.max_y = object[3]

            bbox.label = object[4]
            bbox.probability = 0
            bboxes.boxes.append(bbox)
            cv2.rectangle(
                img, (bbox.min_x, bbox.min_y), (bbox.max_x, bbox.max_y), (255, 0, 0), 4
            )

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.bbox_pub.publish(bboxes)


def main(args=None):
    rclpy.init(args=args)
    detector = Yolov5Detector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
