#!/usr/bin/env python3
import rclpy
import pickle
from rclpy.node import Node

from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Joy
import os

class ReplayRecordings(Node):
    def __init__(self):
        super().__init__('replay_recordings')

        self.path_topic = 'boat_path'
        self.markers_topic = 'state_markers'

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)

        self.declare_parameter('recording_path', "")
        recording_path = str(self.get_parameter('recording_path').value)

        self.declare_parameter('bag_path', "")
        bag_path = str(self.get_parameter('bag_path').value)

        self.path_value = os.path.join(bag_path, recording_path)

        self.joy_control_sub = self.create_subscription(Joy, "joy", self.keyboard_callback, 10)

        self.get_logger().info("press enter to start")

        self.counter = 1
        self.enter_held = False

    def keyboard_callback(self, msg):
        if msg.buttons[1]:
            if not self.enter_held:
                self.enter_held = True
                if self.publish_from_pickles(self.counter):
                    self.get_logger().info(f"proceeding to recording {self.counter}, press enter for next")
                    self.counter += 1
                else:
                    if self.counter == 1:
                        raise Exception("no recordings found!")
                    self.get_logger().info("finished viewing all recordings, press enter to cycle back to 1")
                    self.counter = 1
        else:
            self.enter_held = False
        
    def publish_from_pickles(self, num):
        path_msg = None
        markers_msg = None
        if not os.path.exists(os.path.join(self.path_value, str(num))):
            return False
        with open(os.path.join(self.path_value, str(num), 'path.pkl'), 'rb') as path_file:
            path_msg = pickle.load(path_file)
        with open(os.path.join(self.path_value, str(num), 'markers.pkl'), 'rb') as marker_file:
            markers_msg = pickle.load(marker_file)
        self.path_pub.publish(path_msg)
        self.marker_pub.publish(markers_msg)    
        return True

def main(args=None):
    rclpy.init(args=args)
    replay_recordings = ReplayRecordings()
    rclpy.spin(replay_recordings)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
