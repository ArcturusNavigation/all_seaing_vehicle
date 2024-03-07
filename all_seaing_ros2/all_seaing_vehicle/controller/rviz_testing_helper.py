#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from all_seaing_interfaces.msg import Heartbeat
import rosbag2_py

import datetime


class RvizTestingHelper(Node):

    def __init__(self):
        super().__init__('rviz_testing_helper')

        self.path_topic = 'boat_path'

        self.pub = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.record_odometry, 10)
        self.create_subscription(Heartbeat, "/heartbeat", self.handle_heartbeat, 10)

        self.recorded_poses = []

        self.timestring = datetime.datetime.now().strftime("%I_%M%p_%m_%d_%Y")
        self.counter = 0

        self.last_heartbeat = Heartbeat()
        self.last_heartbeat.e_stopped = True

    def make_new_bag(self, path):
        self.counter += 1

        writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri=f'{self.timestring}_{self.counter}',
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        writer.open(storage_options, converter_options)

        topic_info = rosbag2_py._storage.TopicMetadata(
            name=self.path_topic,
            type='nav_msgs/msg/Path',
            serialization_format='cdr')
        writer.create_topic(topic_info)

        writer.write(
                self.path_topic,
                serialize_message(path),
                self.get_clock().now().nanoseconds)

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
            self.make_new_bag(path)
        else:
            print("no poses detected, skipping publish")

    def handle_heartbeat(self, msg):
        if msg.e_stopped != self.last_heartbeat.e_stopped or msg.in_teleop != self.last_heartbeat.in_teleop:
            if not self.last_heartbeat.e_stopped:
                self.save()
            self.recorded_poses = []
        self.last_heartbeat = msg
        

def main(args=None):
    rclpy.init(args=args)
    rviz_testing_helper = RvizTestingHelper()
    rclpy.spin(rviz_testing_helper)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
