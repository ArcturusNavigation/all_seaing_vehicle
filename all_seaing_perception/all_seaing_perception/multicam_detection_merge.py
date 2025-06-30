#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import LabeledObjectPointCloudArray, LabeledObjectPointCloud
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, TimeSynchronizer

class MulticamDetectionMerge(Node):
    def __init__(self):
        super().__init__("multicam_detection_merge")
        self.detection_sub = Subscriber(self, LabeledObjectPointCloudArray, "refined_object_point_clouds_segments")
        self.detection_back_left_sub = Subscriber(self, LabeledObjectPointCloudArray, "refined_object_point_clouds_segments/back_left")
        self.detection_back_right_sub = Subscriber(self, LabeledObjectPointCloudArray, "refined_object_point_clouds_segments/back_right")
        self.merged_detection_pub = self.create_publisher(LabeledObjectPointCloudArray, "refined_object_point_clouds_segments/merged", 10)

        self.sync = TimeSynchronizer([self.detection_sub, self.detection_back_left_sub, self.detection_back_right_sub], 10)
        self.sync.registerCallback(self.detection_sync_callback)
    
    def detection_sync_callback(self, detections, detections_back_left, detections_back_right):
        # we assume all the detections are on the same frame (base_link or something equivalent)
        merged_detections = LabeledObjectPointCloudArray()
        merged_detections.header = detections.header
        merged_detections.objects.extend(detections.objects)
        merged_detections.objects.extend(detections_back_left.objects)
        merged_detections.objects.extend(detections_back_right.objects)
        self.merged_detection_pub.publish(merged_detections)
        
def main():
    rclpy.init()
    multicam_detection_merge_node = MulticamDetectionMerge()
    rclpy.spin(multicam_detection_merge_node)
    multicam_detection_merge_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()