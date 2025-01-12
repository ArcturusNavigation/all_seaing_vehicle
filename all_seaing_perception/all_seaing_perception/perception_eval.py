#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from all_seaing_interfaces.msg import ObstacleMap, ObstacleCounter, ObstacleCounterArray
import numpy as np

class PerceptionEval(Node):
    """
    counts number of nodes of each color that is seen
    and its local position
    """

    def __init__(self):
        super().__init__("perception_eval")

        self.obs_sub = self.create_subscription(
            ObstacleMap,
            "obstacle_map/labeled",
            self.obs_callback,
            qos_profile_sensor_data,
        )

        self.perception_pub = self.create_publisher(
            ObstacleCounterArray, "obstacle_counter", 10
        )
        self.obstacle_counter = {}
    
    def obs_callback(self, obs_map):
        obstacle_counter = self.obstacle_counter
        for obstacle in obs_map.obstacles:
            # self.get_logger().info(f"Obstacle {obstacle.label} at {obstacle.pose.position.x}, {obstacle.pose.position.y}")

            if str(obstacle.label) in obstacle_counter:
                obstacle_counter[str(obstacle.label)].add(obstacle.id)
            else:
                obstacle_counter[str(obstacle.label)] = {obstacle.id}
        #rosify results
        obstacle_counter_arr_msg = ObstacleCounterArray()
        obstacle_counter_arr_msg.obstacle_counter_array = []
        for label, ids in obstacle_counter.items():
            obstacle_counter_msg = ObstacleCounter()
            obstacle_counter_msg.label = label
            obstacle_counter_msg.obstacle_ids = ids
            obstacle_counter_msg.frequency = len(ids)
            obstacle_counter_arr_msg.obstacle_counter_array.append(obstacle_counter_msg)
        self.perception_pub.publish(obstacle_counter_arr_msg)

def main(args=None):
    rclpy.init(args=args)
    perception_eval = PerceptionEval()
    rclpy.spin(perception_eval)
    perception_eval.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()