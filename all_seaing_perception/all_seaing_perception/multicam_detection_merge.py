#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from custom_time_synchronizers import ResistantTimeSynchronizer

class MulticamDetectionMerge(Node):
    def __init__(self):
        super().__init__("multicam_detection_merge")

        self.declare_parameter("enable_front", True)
        self.declare_parameter("enable_back_left", True)
        self.declare_parameter("enable_back_right", True)
        self.declare_parameter("individual", False)
        self.declare_parameter("approximate", False)
        self.declare_parameter("resistant", True)
        self.declare_parameter("delay", 0.1) # only if not individual and also approximate

        self.enable_front = self.get_parameter("enable_front").get_parameter_value().bool_value
        self.enable_back_left = self.get_parameter("enable_back_left").get_parameter_value().bool_value
        self.enable_back_right = self.get_parameter("enable_back_right").get_parameter_value().bool_value
        self.individual = self.get_parameter("individual").get_parameter_value().bool_value
        self.approximate = self.get_parameter("approximate").get_parameter_value().bool_value
        self.resistant = self.get_parameter("resistant").get_parameter_value().bool_value
        self.delay = self.get_parameter("delay").get_parameter_value().double_value
    
        if not self.individual:
            self.detection_subs = []
            if self.enable_front:
                self.detection_sub = Subscriber(self, ObstacleMap, "detections/front")
                self.detection_subs.append(self.detection_sub)
            if self.enable_back_left:
                self.detection_back_left_sub = Subscriber(self, ObstacleMap, "detections/back_left")
                self.detection_subs.append(self.detection_back_left_sub)
            if self.enable_back_right:
                self.detection_back_right_sub = Subscriber(self, ObstacleMap, "detections/back_right")
                self.detection_subs.append(self.detection_back_right_sub)
            if not self.approximate:
                if not self.resistant:
                    self.sync = TimeSynchronizer(self.detection_subs, 10)
                else:
                    self.sync = ResistantTimeSynchronizer(self.detection_subs, 10, 0.2, True)
            else:
                if not self.resistant:
                    self.sync = ApproximateTimeSynchronizer(self.detection_subs, 10, self.delay)
                else:
                    raise NotImplementedError
            self.sync.registerCallback(self.detection_sync_callback)
        else:
            if self.enable_front:
                self.detection_sub = self.create_subscription(ObstacleMap, "detections/front", self.detection_sync_callback, 10)
            if self.enable_back_left:
                self.detection_sub_back_left = self.create_subscription(ObstacleMap, "detections/back_left", self.detection_sync_callback, 10)
            if self.enable_back_right:
                self.detection_sub_back_right = self.create_subscription(ObstacleMap, "detections/back_right", self.detection_sync_callback, 10)
        
        self.merged_detection_pub = self.create_publisher(ObstacleMap, "detections/merged", 10)
    
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
        self.merged_detection_pub.publish(merged_detections)
        
def main():
    rclpy.init()
    multicam_detection_merge_node = MulticamDetectionMerge()
    rclpy.spin(multicam_detection_merge_node)
    multicam_detection_merge_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()