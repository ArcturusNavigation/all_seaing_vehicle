#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import random

class FindCenter(Node):

    def __init__(self):
        super().__init__('find_center')
        self.publisher_ = self.create_publisher()
