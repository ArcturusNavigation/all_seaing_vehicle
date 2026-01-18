#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class FilterTF(Node):
    def __init__(self):
        super().__init__("filter_tf")
        self.declare_parameter('old_static_tf_topic', '')
        self.old_static_tf_topic = self.get_parameter('old_static_tf_topic').get_parameter_value().string_value
        self.declare_parameter('new_static_tf_topic', '')
        self.new_static_tf_topic = self.get_parameter('new_static_tf_topic').get_parameter_value().string_value
        self.declare_parameter('old_tf_topic', '')
        self.old_tf_topic = self.get_parameter('old_tf_topic').get_parameter_value().string_value
        self.declare_parameter('new_tf_topic', '')
        self.new_tf_topic = self.get_parameter('new_tf_topic').get_parameter_value().string_value
        self.declare_parameter('child_frames_to_remove', [""])
        self.child_frames_to_remove = self.get_parameter('child_frames_to_remove').get_parameter_value().string_array_value
        self.declare_parameter('parent_frames_to_remove', [""])
        self.parent_frames_to_remove = self.get_parameter('parent_frames_to_remove').get_parameter_value().string_array_value
        self.declare_parameter('remap_frames_from', [""])
        self.remap_frames_from = self.get_parameter('remap_frames_from').get_parameter_value().string_array_value
        self.declare_parameter('remap_frames_to', [""])
        self.remap_frames_to = self.get_parameter('remap_frames_to').get_parameter_value().string_array_value
        tf_qos = rclpy.qos.QoSProfile(history = rclpy.qos.HistoryPolicy.KEEP_LAST, 
                                     depth = 10,
                                     reliability = rclpy.qos.ReliabilityPolicy.RELIABLE,
                                     durability = rclpy.qos.DurabilityPolicy.VOLATILE)
        tf_static_qos = rclpy.qos.QoSProfile(history = rclpy.qos.HistoryPolicy.KEEP_LAST, 
                                     depth = 10,
                                     reliability = rclpy.qos.ReliabilityPolicy.RELIABLE,
                                     durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.old_static_tf_sub = self.create_subscription(TFMessage, self.old_static_tf_topic, self.static_tf_cb, tf_static_qos)
        self.new_static_tf_pub = self.create_publisher(TFMessage, self.new_static_tf_topic, tf_static_qos)
        self.old_tf_sub = self.create_subscription(TFMessage, self.old_tf_topic, self.tf_cb, tf_qos)
        self.new_tf_pub = self.create_publisher(TFMessage, self.new_tf_topic, tf_qos)
    
    def tf_cb(self, msg):
        new_tf_message = TFMessage()
        for transform in msg.transforms:
            if transform.child_frame_id not in self.child_frames_to_remove and transform.header.frame_id not in self.parent_frames_to_remove:
                new_transform = transform
                for frame_from, frame_to in zip(self.remap_frames_from, self.remap_frames_to):
                    if transform.header.frame_id == frame_from:
                        new_transform.header.frame_id = frame_to
                    if transform.child_frame_id == frame_from:
                        new_transform.child_frame_id = frame_to
                new_tf_message.transforms.append(new_transform)
        self.new_tf_pub.publish(new_tf_message)
    
    def static_tf_cb(self, msg):
        new_tf_message = TFMessage()
        for transform in msg.transforms:
            if transform.child_frame_id not in self.child_frames_to_remove and transform.header.frame_id not in self.parent_frames_to_remove:
                new_transform = transform
                for frame_from, frame_to in zip(self.remap_frames_from, self.remap_frames_to):
                    if transform.header.frame_id == frame_from:
                        new_transform.header.frame_id = frame_to
                    if transform.child_frame_id == frame_from:
                        new_transform.child_frame_id = frame_to
                new_tf_message.transforms.append(new_transform)
        self.new_static_tf_pub.publish(new_tf_message)

def main(args=None):
    rclpy.init(args=args)
    filter_tf_node = FilterTF()
    rclpy.spin(filter_tf_node)
    filter_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()