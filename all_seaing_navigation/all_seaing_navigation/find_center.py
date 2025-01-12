#!/usr/bin/env python3
import rclpy
import numpy as np
import math

from functools import cmp_to_key
from rclpy.node import Node
from all_seaing_interfaces.msg import ObstacleMap
from nav_msgs.msg import Odometry

class FindCenter(Node):

    def __init__(self):
        super().__init__("find_center")

        self.map_sub = self.create_subscription(
            # Subscribe to something that gets us the two buoy pairs that we need :)
            # Definitely need help from Toya or Janelle, lol
        )
