#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.utils.plotting import Annotator

from all_seaing_interfaces.msg import LabeledBoundingBox2D, LabeledBoundingBox2DArray
from sensor_msgs.msg import Image

import os
import torch
import yaml
import copy
import itertools

class Rectangle:
    # Code from StackOverflow: https://stackoverflow.com/questions/25068538/intersection-and-difference-of-two-rectangles/25068722#25068722
    def intersection(self, other):
        a, b = self, other
        x1 = max(min(a.x1, a.x2), min(b.x1, b.x2))
        y1 = max(min(a.y1, a.y2), min(b.y1, b.y2))
        x2 = min(max(a.x1, a.x2), max(b.x1, b.x2))
        y2 = min(max(a.y1, a.y2), max(b.y1, b.y2))
        if x1<x2 and y1<y2:
            return type(self)(x1, y1, x2, y2)
        else:
            return type(self)(0,0,0,0)
    __and__ = intersection

    def difference(self, other):
        inter = self&other
        if not inter:
            yield self
            return
        xs = {self.x1, self.x2}
        ys = {self.y1, self.y2}
        if self.x1<other.x1<self.x2: xs.add(other.x1)
        if self.x1<other.x2<self.x2: xs.add(other.x2)
        if self.y1<other.y1<self.y2: ys.add(other.y1)
        if self.y1<other.y2<self.y2: ys.add(other.y2)
        for (x1, x2), (y1, y2) in itertools.product(
            pairwise(sorted(xs)), pairwise(sorted(ys))
        ):
            rect = type(self)(x1, y1, x2, y2)
            if rect!=inter:
                yield rect
    __sub__ = difference

    def __init__(self, x1, y1, x2, y2):
        if x1>x2 or y1>y2:
            raise ValueError("Coordinates are invalid")
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2

    def __iter__(self):
        yield self.x1
        yield self.y1
        yield self.x2
        yield self.y2

    def __eq__(self, other):
        return isinstance(other, Rectangle) and tuple(self)==tuple(other)
    def __ne__(self, other):
        return not (self==other)

    def __repr__(self):
        return type(self).__name__+repr(tuple(self))
    
    def area(self):
        return (self.x2-self.x1) * (self.y2-self.y1)


def pairwise(iterable):
    # https://docs.python.org/dev/library/itertools.html#recipes
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


class Yolov11_Beacon_Node(Node):

    def __init__(self):
        super().__init__("yolov11_beacon_node")

        perception_prefix = get_package_share_directory("all_seaing_perception")
        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        # Declare/get ROS parameters
        model_name = self.declare_parameter(
            "model", "beacons_best").get_parameter_value().string_value
        self.device = self.declare_parameter(
            "device", "default").get_parameter_value().string_value
        self.conf = self.declare_parameter(
            "conf", 0.6).get_parameter_value().double_value
        label_config = self.declare_parameter(
            "label_config", "color_label_mappings").get_parameter_value().string_value
        self.use_color_names = self.declare_parameter(
            "use_color_names", True).get_parameter_value().bool_value
        
        self.filter_beacon_indicators = self.declare_parameter(
            "filter_beacon_indicators", False).get_parameter_value().bool_value
        
        self.beacon_filter_ratio = self.declare_parameter(
            "beacon_filter_ratio", 0.2).get_parameter_value().double_value
        
        self.indicator_to_beacon_bbox = self.declare_parameter(
            "indicator_to_beacon_bbox", False).get_parameter_value().bool_value
        

        if self.device == "default":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        self.get_logger().info(f'YOLO DEVICE USED: {self.device}')

        label_config = self.get_parameter("label_config").get_parameter_value().string_value
        yaml_file_path = os.path.join(bringup_prefix, "config","perception", label_config + ".yaml")
        
        with open(yaml_file_path,"r") as f:
            self.label_dict = yaml.safe_load(f)
        
        self.indicator_labels = [self.label_dict[name] for name in ["green_indicator", "red_indicator"]]
        self.beacon_label = self.label_dict["beacon"]

        # Get the model's path
        engine_path = os.path.join(perception_prefix, "models", model_name + ".engine")
        pt_path = os.path.join(perception_prefix, "models", model_name + ".pt")

        # Initialize YOLO model
        # check if we are using the jetson and if the engine path exists.
        self.cv_bridge = CvBridge()
        if os.path.exists("/etc/nv_tegra_release") and os.path.isfile(engine_path):
            self.get_logger().info(f"Loading model from tensorRT engine: {engine_path}")
            self.model = YOLO(engine_path)
        elif os.path.isfile(pt_path):
            self.get_logger().info(f"Loading model from pt model: {pt_path}")
            self.model = YOLO(pt_path)
        else:
            self.get_logger().error(f"Both model paths do not exist :( TensorRT: {engine_path} and pt: {pt_path}")

        # Setup QoS profile
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Publisher and Subscriber
        self._pub = self.create_publisher(LabeledBoundingBox2DArray, "bounding_boxes", 10)
        self._image_pub = self.create_publisher(Image, "annotated_image", 10)
        self._sub = self.create_subscription(Image, "image", self.image_cb, image_qos_profile)

    def overlap_ratio(self, bbox1: LabeledBoundingBox2D, bbox2: LabeledBoundingBox2D):
        """
        Computing ratio of overlap of bbox1 & bbox2 / bbox1 area
        """
        rect1 = Rectangle(bbox1.min_x, bbox1.min_y, bbox1.max_x, bbox1.max_y)
        rect2 = Rectangle(bbox2.min_x, bbox2.min_y, bbox2.max_x, bbox2.max_y)
        return (rect1&rect2).area()/rect1.area()

    def image_cb(self, msg: Image) -> None:

        # Convert image to cv_image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        annotator = Annotator(cv_image, 2, 2, "Arial.ttf", False)

        pred_results = self.model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=self.conf,
            device=self.device
        )
        results: Results = pred_results[0].cpu()

        # if self.filter_beacon_indicators:
        pending_bounding_box_msgs: list[LabeledBoundingBox2D] = []
        beacon_bboxes: list[LabeledBoundingBox2D] = []
        
        labeled_bounding_box_msgs = LabeledBoundingBox2DArray()
        labeled_bounding_box_msgs.header = msg.header

        for box_data in results.boxes:

            box_msg = LabeledBoundingBox2D()

            if results.boxes:

                box_msg.probability = float(box_data.conf)
                center_x, center_y, width, height = box_data.xywh[0]
                box_msg.min_x = int(center_x - width / 2)
                box_msg.max_x = int(center_x + width / 2)
                box_msg.min_y = int(center_y - height / 2)
                box_msg.max_y = int(center_y + height / 2)
                label_name = self.model.names[int(box_data.cls)]

                # if not self.filter_beacon_indicators:
                #     labeled_bounding_box_msgs.boxes.append(box_msg)
                # else:
                if box_data.cls in self.indicator_labels:
                    pending_bounding_box_msgs.append(box_msg)
                else:
                    labeled_bounding_box_msgs.boxes.append(box_msg)
                
                if box_data.cls == self.beacon_label:
                    beacon_bboxes.append(box_msg)

                class_name = f"{label_name}_{str(int(box_data.cls))}"
                class_name_list = class_name.split("_")
                color_name = class_name_list[0] # assumes labels follow "color_object" format
                color = ()
                text_color = (0,0,0)
                if color_name == "red":
                    color = (0, 0, 255)
                elif color_name == "green":
                    color = (0,255,0)
                elif color_name == "blue":
                    color = (255, 0, 0)
                elif color_name == "yellow":
                    color = (0,230,230)
                elif color_name == "black":
                    color = (0,0,0)
                elif color_name == "white":
                    color = (255,255,255)
                else: 
                    color = (128,128,128)

                color_label = -1
                if color_name in self.label_dict:
                    color_label = self.label_dict[color_name]
                    # if color_name in ["black", "blue", "red", "green"]:
                    #     object_name = class_name_list[1]
                    #     if "buoy" not in object_name:
                    #         color_label = -1

                    annotator.box_label((box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y), class_name, color, text_color)
                    self.get_logger().debug(f"Detected: {class_name} Msg Label is {box_msg.label}")
                else: 
                    box_msg.label = -1 # for misc things
                    annotator.box_label((box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y), class_name, color, text_color)
                    self.get_logger().debug(f"Detected: {class_name} Item not in label_dict")

                if self.use_color_names:
                    box_msg.label = color_label
                else:
                    box_msg.label = int(box_data.cls)

        # if self.filter_beacon_indicators:
        for indicator_box in pending_bounding_box_msgs:
            added = False
            for beacon_box in beacon_bboxes:
                if self.overlap_ratio(indicator_box, beacon_box) > self.beacon_filter_ratio:
                    if self.indicator_to_beacon_bbox:
                        new_indicator = copy.deepcopy(beacon_box)
                        new_indicator.label = indicator_box.label
                        # beacon_box.label = indicator_box.label
                        labeled_bounding_box_msgs.boxes.append(new_indicator)
                    else:
                        labeled_bounding_box_msgs.boxes.append(indicator_box)
                    # self.get_logger().info(f"FOUND INDICATOR {indicator_box.label}")
                    added = True
                    break
            if not self.filter_beacon_indicators and not added:
                labeled_bounding_box_msgs.boxes.append(indicator_box)

        # Publish detections
        self._pub.publish(labeled_bounding_box_msgs)

        # Convert annotated image back to ROS Image message
        annotated_frame = annotator.result()  
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self._image_pub.publish(annotated_image_msg)  # Publish annotated image


def main():
    rclpy.init()
    node = Yolov11_Beacon_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
