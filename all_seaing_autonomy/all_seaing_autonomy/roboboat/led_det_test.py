#!/usr/bin/env python3
from ast import Num
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


from all_seaing_interfaces.msg import ObstacleMap, Obstacle, LabeledBoundingBox2DArray, LabeledBoundingBox2D
from all_seaing_interfaces.action import FollowPath, Task
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from tf_transformations import euler_from_quaternion

import os
import yaml
import math
import time
from collections import deque

TIMER_PERIOD = 1 / 60


class Bbox:
    def __init__(self, bbox_msg):
        self.x = bbox_msg.min_x
        self.y = bbox_msg.min_y
        self.w = bbox_msg.max_x-bbox_msg.min_x
        self.h = bbox_msg.max_y-bbox_msg.min_y


class LEDTest(Node):
    def __init__(self):
        super().__init__("LEDTest")

        # Subscribers and publishers
        self.seg_image_bbox_sub = self.create_subscription(
            LabeledBoundingBox2DArray, "bounding_boxes_ycrcb", self.seg_bbox_cb, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_cb, 10
        )

        self.image_size = (400,400)
        self.seg_bboxes = deque()
        self.max_seg_bboxes = 10 # guarantee this is even

        self.red_labels = set()
        self.green_labels = set()

        self.changed = False

        self.red_labels.add(17)
        self.green_labels.add(11)

    def camera_info_cb(self, msg):
        '''
        Gets camera image info from all_seaing_perception.
        '''
        self.image_size = (msg.width, msg.height)

    def seg_bbox_cb(self, msg):
        '''
        Handles when an color segmented image gets published
        '''

        self.seg_bboxes.append(msg.boxes)
        if len(self.seg_bboxes) > self.max_seg_bboxes:
            self.seg_bboxes.popleft()
        else:
            return

        all_seg_bboxes = list(self.seg_bboxes)

        redBboxes = []
        greenBboxes = []
        for frame in all_seg_bboxes: #fixed time, frame= LabeledBoundingBox2D[]
            redBboxes.append([])
            greenBboxes.append([])
            for box in frame:
                if box.label in self.red_labels:
                    redBboxes[-1].append(Bbox(box))
                if box.label in self.green_labels:
                    greenBboxes[-1].append(Bbox(box))

        cutoff = len(all_seg_bboxes)//2
        beforeRed = redBboxes[0:cutoff]
        afterRed = redBboxes[cutoff:len(redBboxes)]
        beforeGreen = greenBboxes[0:cutoff]
        afterGreen = greenBboxes[cutoff:len(greenBboxes)]

        # TODO: FIX MAGIC NUMBER TERRITORY
        epsilon = 0.02*math.sqrt(self.image_size[0]**2 + self.image_size[1]**2)
        lmbda = epsilon
        p = 5
        limit = 0.5

        if self.changed or self.led_changed(beforeRed, afterRed, beforeGreen, afterGreen, epsilon, lmbda, p, limit):
            self.get_logger().info("LED detection: color changed")
            self.changed = True
        else:
            self.get_logger().info("LED detection: no change")
    
    # robust realtime visual signal processing
    def led_changed(self, beforeRed, afterRed, beforeGreen, afterGreen, epsilon, lmbda, p, limit):
        '''
        beforeRed :: [[Bbox]] | len(beforeRed) > 0
        frames with red bounding boxes before signal event

        beforeGreen :: [[Bbox]] | len(beforeGreen) = len(beforeRed) > 0
        same as `beforeRed` but for green bounding boxes

        afterGreen :: [[Bbox]] | len(afterGreen) = len(afterRed) > 0
        same as `afterRed` but for green bounding boxes

        epsilon :: Num
        maximum distance at which two bounding boxes are considered the same;
        also minimum distance needed to identify bounding boxes as unique

        lmda :: Num
        maximum deviation in position of a bounding box across frames

        p :: Int
        number of frames to sample from full list to determine main bounding boxes;
        should be small for performance reasons

        limit :: Prob
        threshold probability of existence required to detect a change in bounding box color

        → Bool
        whether signal has changed

        ---

        Bbox := {
            x :: Num
            y :: Num
            w :: Num | w > 0
            h :: Num | h > 0
        }

        Prob := Num a | 0 <= a <= 1
        '''
        fmap = lambda f, l: [f(x) for x in l]  # A → B → [A] → [B]
        flatten = lambda l: l[0] + flatten(l[1:]) if len(l) > 0 else []  # [[A]] → [A]
        filter = lambda p, l: [x for x in l if p(x)]  # (A → Bool) → [A] → [A]
        product = lambda a, b: flatten([[(alpha, beta) for alpha in a] for beta in b])  # [A] → [B] → [(A, B)]
        access = lambda l1, l2: [l1[i] for i in l2]  # [A] → [Int] → [A]
        distinguishable = lambda dist: dist > epsilon  # Num → Bool
        matching = lambda dist: dist < lmbda  # Num → Bool
        changed = lambda prob: prob >= limit  # Prob → Bool
        multiply = lambda x: x[0] * x[1]  # (Num, Num) → Num
        norm = lambda b1: lambda b2: math.sqrt((b1.x - b2.x) ** 2 + (b1.y - b2.y) ** 2)  # Num → (Num → Num)
        estimator = lambda trials: sum(trials) / len(trials)  # [Bool] → Num

        # [Bbox] → (Bbox → Bool)
        # whether the bounding box is distinguishable from all bounding boxes in a list of bounding boxes
        distinct = lambda boxes: lambda box: all(fmap(distinguishable, fmap(norm(box), boxes)))

        # Int | nBefore > 0
        # number of "before" frames
        nBefore = len(beforeRed)

        # Int | nAfter > 0
        # number of "after" frames
        nAfter = len(afterGreen)

        # Int → Int → Int | n > 0 → [Int]
        # generate `n` evenly spaced elements from an arbitrary discrete range
        linspace = lambda a, b, n: [a] + linspace(a + (b-a)//(n-1), b, n-1) if n > 1 else [a]

        # [Int]
        # indices of "before" frames to sample
        beforeSampleIndices = fmap(int, linspace(0, nBefore-1, p))

        # ([Bbox], [Bbox]) → [Bbox]
        # remove first bounding boxes that are indistinguishable from second ones in the same frame
        filterFrame = lambda colors: filter(distinct(colors[1]), colors[0])

        # [[Bbox]] → [[Bbox]]
        # remove red bounding boxes that are indistinguishable from green ones for each frame
        beforeRedCandidates = fmap(filterFrame, zip(beforeRed, beforeGreen))
        # [[Bbox]]
        # bounding box data for red before frames
        beforeSampleRed = access(beforeRedCandidates, beforeSampleIndices)

        # [Bbox]
        # list of identified red bounding boxes from sample frames
        redBboxes = flatten(beforeSampleRed)

        # Bbox → ([Bbox] → Bool)
        # detect whether a particular bounding box exists in a different frame
        bboxExists = lambda bbox: lambda frame: any(fmap(matching, fmap(norm(bbox), frame)))

        # Bbox → [[Bbox]] → Prob
        # determine probability that a given bounding box exists in time series
        probExistence = lambda box, series: estimator(fmap(bboxExists(box), series))

        # [Int]
        # indices of "after" frames to sample
        afterSampleIndices = fmap(int, linspace(0, nAfter-1, p))

        # [[Bbox]]
        # bounding box data for green after frames
        afterSampleGreen = access(afterGreen, afterSampleIndices)

        # [Bbox]
        # list of identified green bounding boxes from sample frames
        greenBoxes = flatten(afterSampleGreen)

        # (Bbox, Bbox) → Bool
        # check for overlap between bounding box set and a single bounding box
        overlapping = lambda boxes: matching(norm(boxes[0])(boxes[1]))

        # [(Bbox, Bbox)]
        # list of potential candidates for a signal switch
        signalCandidates = filter(overlapping, product(redBboxes, greenBoxes))

        # (Bbox, Bbox) → Prob
        # probability of specific candidate pair existing
        candidateProbability = lambda boxes: (probExistence(boxes[0], beforeRed), probExistence(boxes[1], afterGreen))

        # [(Prob, Prob)]
        # probability of existence of each bounding box in signal candidates
        probabilities = fmap(candidateProbability, signalCandidates)

        # [Prob]
        # probability of existence of each bounding box pair in signal candidates
        pairProbabilities = fmap(multiply, probabilities)

        # check for any probability exceeding confidence threshold
        return any(fmap(changed, pairProbabilities))

def main(args=None):
    rclpy.init(args=args)
    test = LEDTest()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
