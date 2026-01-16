#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap, LabeledObjectPointCloudArray, Obstacle, LabeledObjectPointCloud
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from typing import List
import numpy as np

class MulticamDetectionMerge(Node):
    def __init__(self):
        super().__init__("multicam_detection_merge")

        self.declare_parameter("enable_front", True)
        self.declare_parameter("enable_back_left", True)
        self.declare_parameter("enable_back_right", True)
        self.declare_parameter("individual", False)
        self.declare_parameter("approximate", False)
        self.declare_parameter("delay", 0.1) # only if not individual and also approximate
        self.declare_parameter("duplicate_thres", 0.15)
        self.declare_parameter("angle_thres", 10.0)

        self.enable_front = self.get_parameter("enable_front").get_parameter_value().bool_value
        self.enable_back_left = self.get_parameter("enable_back_left").get_parameter_value().bool_value
        self.enable_back_right = self.get_parameter("enable_back_right").get_parameter_value().bool_value
        self.individual = self.get_parameter("individual").get_parameter_value().bool_value
        self.approximate = self.get_parameter("approximate").get_parameter_value().bool_value
        self.delay = self.get_parameter("delay").get_parameter_value().double_value
        self.duplicate_thres = self.get_parameter("duplicate_thres").get_parameter_value().double_value
        self.angle_thres = self.get_parameter("angle_thres").get_parameter_value().double_value
    
        if not self.individual:
            self.detection_subs = []
            self.pcl_subs = []
            if self.enable_front:
                self.detection_sub = Subscriber(self, ObstacleMap, "detections/front")
                self.pcl_sub = Subscriber(self, LabeledObjectPointCloudArray, "labeled_object_point_clouds/front")
                self.detection_subs.append(self.detection_sub)
                self.pcl_subs.append(self.pcl_sub)
            if self.enable_back_left:
                self.detection_back_left_sub = Subscriber(self, ObstacleMap, "detections/back_left")
                self.pcl_back_left_sub = Subscriber(self, LabeledObjectPointCloudArray, "labeled_object_point_clouds/back_left")
                self.detection_subs.append(self.detection_back_left_sub)
                self.pcl_subs.append(self.pcl_back_left_sub)
            if self.enable_back_right:
                self.detection_back_right_sub = Subscriber(self, ObstacleMap, "detections/back_right")
                self.pcl_back_right_sub = Subscriber(self, LabeledObjectPointCloudArray, "labeled_object_point_clouds/back_right")
                self.detection_subs.append(self.detection_back_right_sub)
                self.pcl_subs.append(self.pcl_back_right_sub)
            if not self.approximate:
                self.sync = TimeSynchronizer(self.detection_subs, 10)
                self.pcl_sync = TimeSynchronizer(self.pcl_subs, 10)
            else:
                self.sync = ApproximateTimeSynchronizer(self.detection_subs, 10, self.delay)
                self.pcl_sync = ApproximateTimeSynchronizer(self.pcl_subs, 10, self.delay)
            self.sync.registerCallback(self.detection_sync_callback)
            self.pcl_sync.registerCallback(self.pcl_sync_callback)
        else:
            if self.enable_front:
                self.detection_sub = self.create_subscription(ObstacleMap, "detections/front", self.detection_sync_callback, 10)
                self.pcl_sub = self.create_subscription(LabeledObjectPointCloudArray, "labeled_object_point_clouds/front", self.pcl_sync_callback, 10)
            if self.enable_back_left:
                self.detection_sub_back_left = self.create_subscription(ObstacleMap, "detections/back_left", self.detection_sync_callback, 10)
                self.pcl_sub_back_left = self.create_subscription(LabeledObjectPointCloudArray, "labeled_object_point_clouds/back_left", self.pcl_sync_callback, 10)
            if self.enable_back_right:
                self.detection_sub_back_right = self.create_subscription(ObstacleMap, "detections/back_right", self.detection_sync_callback, 10)
                self.pcl_sub_back_right= self.create_subscription(LabeledObjectPointCloudArray, "labeled_object_point_clouds/back_right", self.pcl_sync_callback, 10)
        self.merged_detection_pub = self.create_publisher(ObstacleMap, "detections/merged", 10)
        self.merged_pcl_pub = self.create_publisher(LabeledObjectPointCloudArray, "labeled_object_point_clouds/merged", 10)
    
    def get_angle_point(self, pt: Point):
        return np.arctan2(pt.y, pt.x)
    
    def get_angle_obs(self, obs: Obstacle):
        return self.get_angle_point(obs.local_point.point)
    
    def angle_diff(self, angle1, angle2):
        mod1 = angle1 % (2*np.pi)
        mod2 = angle2 % (2*np.pi)
        diff_abs = abs(mod1 - mod2)
        return min(diff_abs, 2*np.pi - diff_abs)

    def angle_diff_obs(self, obs1: Obstacle, obs2: Obstacle):
        return self.angle_diff(self.get_angle_obs(obs1), self.get_angle_obs(obs2))
    
    def pos_diff(self, pos1: Point, pos2: Point):
        return np.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)
    
    def pos_diff_obs(self, obs1: Obstacle, obs2: Obstacle):
        return self.pos_diff(obs1.local_point.point, obs2.local_point.point)
    
    def unique_obstacles(self, obstacles: List[Obstacle]):
        obstacles.sort(key=self.get_angle_obs)
        start_ptr = 0
        while start_ptr < len(obstacles):
            end_ptr = start_ptr + 1
            remove_current = False
            while end_ptr < start_ptr + len(obstacles) and self.angle_diff_obs(obstacles[end_ptr % len(obstacles)], obstacles[start_ptr]) < self.angle_thres:
                # self.get_logger().info(f'DISTANCE: {self.pos_diff_obs(obstacles[end_ptr % len(obstacles)], obstacles[start_ptr])}')
                if self.pos_diff_obs(obstacles[end_ptr % len(obstacles)], obstacles[start_ptr]) < self.duplicate_thres:
                    # self.get_logger().info(f'AREAS: {obstacles[start_ptr].polygon_area}, {obstacles[end_ptr % len(obstacles)].polygon_area}')
                    if obstacles[start_ptr].polygon_area > obstacles[end_ptr % len(obstacles)].polygon_area:
                        if end_ptr >= len(obstacles):
                            start_ptr -= 1
                        obstacles.pop(end_ptr % len(obstacles))
                    else:
                        # self.get_logger().info(f'REMOVE CURRENT ONE')
                        remove_current = True
                        break
                else:
                    end_ptr += 1
            if not remove_current:
                start_ptr += 1
            else:
                obstacles.pop(start_ptr)

    def get_centroid(self, pcl: LabeledObjectPointCloud) -> Point:
        ctr_np = pc2.read_points_numpy(pcl.cloud).mean(axis=0)
        return Point(x=ctr_np[0].item(), y=ctr_np[1].item(), z=ctr_np[2].item())

    def get_angle_pcl(self, pcl: LabeledObjectPointCloud):
        return self.get_angle_point(self.get_centroid(pcl))
    
    def angle_diff_pcl(self, pcl1: LabeledObjectPointCloud, pcl2: LabeledObjectPointCloud):
        return self.angle_diff(self.get_angle_pcl(pcl1), self.get_angle_pcl(pcl2))
    
    def pos_diff_pcl(self, pcl1: LabeledObjectPointCloud, pcl2: LabeledObjectPointCloud):
        return self.pos_diff(self.get_centroid(pcl1), self.get_centroid(pcl2))

    def unique_pcls(self, pcls: List[LabeledObjectPointCloud]):
        pcls.sort(key=self.get_angle_pcl)
        start_ptr = 0
        while start_ptr < len(pcls):
            end_ptr = start_ptr + 1
            remove_current = False
            while end_ptr < start_ptr + len(pcls) and self.angle_diff_pcl(pcls[end_ptr % len(pcls)], pcls[start_ptr]) < self.angle_thres:
                if self.pos_diff_pcl(pcls[end_ptr % len(pcls)], pcls[start_ptr]) < self.duplicate_thres:
                    pcls.pop(end_ptr % len(pcls))
                    # self.get_logger().info(f'SIZES: {pcls[start_ptr].cloud.width}, {pcls[end_ptr % len(pcls)].cloud.width}')
                    if pcls[start_ptr].cloud.width > pcls[end_ptr % len(pcls)].cloud.width:
                        if end_ptr >= len(pcls):
                            start_ptr -= 1
                        pcls.pop(end_ptr % len(pcls))
                    else:
                        # self.get_logger().info(f'REMOVE CURRENT ONE')
                        remove_current = True
                        break
                else:
                    end_ptr += 1
            if not remove_current:
                start_ptr += 1
            else:
                pcls.pop(start_ptr)
    
    def detection_sync_callback(self, *args):
        # we assume all the detections are on the same frame (base_link or something equivalent)
        merged_detections = ObstacleMap()
        detections: ObstacleMap
        for detections in args:
            merged_detections.header = detections.header
            merged_detections.local_header = detections.local_header
            merged_detections.ns = detections.ns
            merged_detections.is_labeled = detections.is_labeled
            
            merged_detections.obstacles.extend(detections.obstacles)
        # self.get_logger().info(f'OBJECTS BEFORE DUPLICATE CLEARANCE: {len(merged_detections.obstacles)}')
        self.unique_obstacles(merged_detections.obstacles)
        # self.get_logger().info(f'OBJECTS AFTER DUPLICATE CLEARANCE: {len(merged_detections.obstacles)}')
        self.merged_detection_pub.publish(merged_detections)

    def pcl_sync_callback(self, *args):
        merged_pcls = LabeledObjectPointCloudArray()
        pcls: LabeledObjectPointCloudArray
        for pcls in args:
            merged_pcls.header = pcls.header  
            merged_pcls.objects.extend(pcls.objects)
        self.unique_pcls(merged_pcls.objects)
        self.merged_pcl_pub.publish(merged_pcls)
        
def main():
    rclpy.init()
    multicam_detection_merge_node = MulticamDetectionMerge()
    rclpy.spin(multicam_detection_merge_node)
    multicam_detection_merge_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()