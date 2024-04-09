#!/usr/bin/env python3
import rclpy
import pickle
from rclpy.node import Node
from rclpy.serialization import serialize_message

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Vector3
from all_seaing_interfaces.msg import Heartbeat, ASV2State
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
import rosbag2_py
import os
from rclpy.time import Duration

import datetime


class RvizTestingHelper(Node):

    def __init__(self):
        super().__init__('rviz_testing_helper')

        self.path_topic = 'boat_path'
        self.markers_topic = 'state_markers'

        self.declare_parameter('bag_path', None)
        bag_path_value = self.get_parameter('bag_path').value
        self.bag_path = str(bag_path_value) if bag_path_value else None

        self.pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.record_odometry, 10)
        self.create_subscription(Heartbeat, "/heartbeat", self.handle_heartbeat, 10)
        self.create_subscription(ASV2State, "/boat_state", self.update_markers, 10)

        self.recorded_poses = []
        self.recorded_markers = []

        self.timestring = datetime.datetime.now().strftime("%I_%M%p_%m_%d_%Y")
        self.counter = 0

        self.last_heartbeat = Heartbeat()
        self.last_heartbeat.e_stopped = True

        self.last_state = None
        self.marker_id_counter = 0
        self.last_num_markers_used = 0

    def update_markers(self, msg):
        if not self.recorded_poses:
            return
        if msg.current_state != self.last_state:
            self.marker_id_counter += 1
            marker = Marker()
            marker.header = self.recorded_poses[-1].header
            marker.ns = "state_markers"
            marker.id = self.marker_id_counter
            marker.type = Marker.TEXT_VIEW_FACING
            marker.pose = self.recorded_poses[-1].pose
            marker.scale = Vector3()
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color = ColorRGBA()
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=0).to_msg()
            marker.text = msg.current_state
            
            self.recorded_markers.append(marker)
            self.last_state = msg.current_state

    def make_new_bag(self, path, markers):
        self.counter += 1
    
        fullpath = os.path.join(self.bag_path, self.timestring, str(self.counter))
        if not os.path.exists(fullpath):
            os.makedirs(fullpath)

        with open(os.path.join(fullpath, 'path.pkl'), 'wb') as file:
            pickle.dump(path, file)
        with open(os.path.join(fullpath, 'markers.pkl'), 'wb') as file:
            pickle.dump(markers, file)

    def record_odometry(self, msg):
        if self.last_heartbeat.e_stopped:
            return

        pose_stamped = PoseStamped()
        pose_stamped.pose = msg.pose.pose
        pose_stamped.header = msg.header
        self.recorded_poses.append(pose_stamped)

    def save(self):
        print("publishing recording")
        if self.recorded_poses:
            path = Path()
            path.poses = self.recorded_poses
            path.header = self.recorded_poses[-1].header

            self.pub.publish(path)

            marker_array = MarkerArray()
            deleteAllMarker = Marker()
            deleteAllMarker.header = self.recorded_poses[-1].header
            deleteAllMarker.action = Marker.DELETEALL
            marker_array.markers = [deleteAllMarker]
            marker_array.markers.extend(self.recorded_markers)

            self.marker_pub.publish(marker_array)
            self.last_num_markers_used = self.marker_id_counter

            if self.bag_path is not None:
                self.make_new_bag(path, marker_array)
        else:
            print("no poses detected, skipping publish")

    def handle_heartbeat(self, msg):
        if msg.e_stopped != self.last_heartbeat.e_stopped or msg.in_teleop != self.last_heartbeat.in_teleop:
            if not self.last_heartbeat.e_stopped:
                self.save()
            self.recorded_poses = []
            self.recorded_markers = []
            self.marker_id_counter = 0
            self.last_state = None
        self.last_heartbeat = msg
        

def main(args=None):
    rclpy.init(args=args)
    rviz_testing_helper = RvizTestingHelper()
    rclpy.spin(rviz_testing_helper)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
