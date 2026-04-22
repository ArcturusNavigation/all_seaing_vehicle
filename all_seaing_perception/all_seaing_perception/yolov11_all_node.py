#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

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
import time
import multiprocessing as mp
import numpy as np


class Rectangle:
    def intersection(self, other):
        a, b = self, other
        x1 = max(min(a.x1, a.x2), min(b.x1, b.x2))
        y1 = max(min(a.y1, a.y2), min(b.y1, b.y2))
        x2 = min(max(a.x1, a.x2), max(b.x1, b.x2))
        y2 = min(max(a.y1, a.y2), max(b.y1, b.y2))
        if x1 < x2 and y1 < y2:
            return type(self)(x1, y1, x2, y2)
        else:
            return type(self)(0, 0, 0, 0)
    __and__ = intersection

    def dist(self, other):
        if abs(self.x - other.x) <= (self.w + other.w):
            dx = 0
        else:
            dx = abs(self.x - other.x) - (self.w + other.w)
        if abs(self.y - other.y) <= (self.h + other.h):
            dy = 0
        else:
            dy = abs(self.y - other.y) - (self.h + other.h)
        return dx + dy

    def difference(self, other):
        inter = self & other
        if not inter:
            yield self
            return
        xs = {self.x1, self.x2}
        ys = {self.y1, self.y2}
        if self.x1 < other.x1 < self.x2: xs.add(other.x1)
        if self.x1 < other.x2 < self.x2: xs.add(other.x2)
        if self.y1 < other.y1 < self.y2: ys.add(other.y1)
        if self.y1 < other.y2 < self.y2: ys.add(other.y2)
        for (x1, x2), (y1, y2) in itertools.product(
            pairwise(sorted(xs)), pairwise(sorted(ys))
        ):
            rect = type(self)(x1, y1, x2, y2)
            if rect != inter:
                yield rect
    __sub__ = difference

    def __init__(self, x1, y1, x2, y2):
        if x1 > x2 or y1 > y2:
            raise ValueError("Coordinates are invalid")
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.x = (x1 + x2) / 2.0
        self.y = (y1 + y2) / 2.0
        self.w = (self.x2 - self.x1) / 2.0
        self.h = (self.y2 - self.y1) / 2.0

    def __iter__(self):
        yield self.x1
        yield self.y1
        yield self.x2
        yield self.y2

    def __eq__(self, other):
        return isinstance(other, Rectangle) and tuple(self) == tuple(other)
    def __ne__(self, other):
        return not (self == other)
    def __repr__(self):
        return type(self).__name__ + repr(tuple(self))
    def area(self):
        return (self.x2 - self.x1) * (self.y2 - self.y1)


def pairwise(iterable):
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def model_worker(index: int, model_path: str, conf: float, device: str,
                 in_queue: mp.Queue, out_queue: mp.Queue):
    """
    Runs in its own process. Owns a single YOLO model and its CUDA context.
    Waits for image arrays on in_queue, sends back raw box data on out_queue.
    Receives None as a sentinel to shut down.
    """
    model = YOLO(model_path)

    while True:
        item = in_queue.get()
        if item is None:  # shutdown sentinel
            break

        frame_id, cv_image = item
        t0 = time.time()

        pred_results = model.predict(
            source=cv_image,
            verbose=False,
            stream=False,
            conf=conf,
            device=device,
        )

        t1 = time.time()

        results: Results = pred_results[0].cpu()
        boxes = []
        for box_data in results.boxes:
            boxes.append({
                "conf":  float(box_data.conf),
                "xywh":  box_data.xywh[0].tolist(),
                "cls":   int(box_data.cls),
                "name":  model.names[int(box_data.cls)],
            })

        out_queue.put((frame_id, index, boxes, t1 - t0))


class YOLOv11Node(Node):

    def __init__(self):
        super().__init__("yolov11_all_node")

        perception_prefix = get_package_share_directory("all_seaing_perception")
        bringup_prefix = get_package_share_directory("all_seaing_bringup")

        model_names = self.declare_parameter(
            "models", ["best"]).get_parameter_value().string_array_value
        label_offsets = self.declare_parameter(
            "label_offsets", [0]).get_parameter_value().integer_array_value
        self.device = self.declare_parameter(
            "device", "default").get_parameter_value().string_value
        confs = self.declare_parameter(
            "confs", [0.6]).get_parameter_value().double_array_value
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
        self.match_indicators_banners = self.declare_parameter(
            "match_indicators_banners", False).get_parameter_value().bool_value
        self.indicator_banner_px_dist = self.declare_parameter(
            "indicator_banner_px_dist", 0).get_parameter_value().integer_value
        self.ignore_indicator_filters = self.declare_parameter(
            "ignore_indicator_filters", False).get_parameter_value().bool_value

        if self.device == "default":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.get_logger().info(f"YOLO DEVICE USED: {self.device}")

        yaml_file_path = os.path.join(
            bringup_prefix, "config", "perception", label_config + ".yaml"
        )
        with open(yaml_file_path, "r") as f:
            self.label_dict = yaml.safe_load(f)

        self.inv_label_dict = {v: k for k, v in self.label_dict.items()}

        if not self.ignore_indicator_filters:
            self.indicator_labels = [self.label_dict[n] for n in ["green_indicator", "red_indicator"]]
            self.green_indicator_labels = [self.label_dict["green_indicator"]]
            self.red_indicator_labels = [self.label_dict["red_indicator"]]
            self.beacon_label = self.label_dict["beacon"]
            self.number_labels = [self.label_dict[n] for n in ["number_1", "number_2", "number_3"]]

        self.label_offsets = list(label_offsets)
        self.cv_bridge = CvBridge()

        # ------------------------------------------------------------------
        # Spawn one worker process per model
        # ------------------------------------------------------------------
        self._in_queues: list[mp.Queue] = []
        self._out_queue: mp.Queue = mp.Queue()   # all workers share one output queue
        self._workers: list[mp.Process] = []
        self._num_models = len(model_names)

        for i, (model_name, conf) in enumerate(zip(model_names, confs)):
            engine_path = os.path.join(perception_prefix, "models", model_name + ".engine")
            pt_path = os.path.join(perception_prefix, "models", model_name + ".pt")

            if os.path.exists("/etc/nv_tegra_release") and os.path.isfile(engine_path):
                model_path = engine_path
                self.get_logger().info(f"[model {i}] loading TensorRT engine: {engine_path}")
            elif os.path.isfile(pt_path):
                model_path = pt_path
                self.get_logger().info(f"[model {i}] loading pt model: {pt_path}")
            else:
                self.get_logger().error(f"[model {i}] no model found at {engine_path} or {pt_path}")
                model_path = None

            in_q: mp.Queue = mp.Queue(maxsize=1)  # maxsize=1 drops stale frames
            self._in_queues.append(in_q)

            p = mp.Process(
                target=model_worker,
                args=(i, model_path, conf, self.device, in_q, self._out_queue),
                daemon=True,
            )
            p.start()
            self._workers.append(p)

        self._frame_id = 0  # monotonic counter to match responses to requests

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, # should be BEST_EFFORT for IRL?
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._pub = self.create_publisher(LabeledBoundingBox2DArray, "bounding_boxes", 10)
        self._image_pub = self.create_publisher(Image, "annotated_image", 10)
        self._sub = self.create_subscription(Image, "image", self.image_cb, image_qos_profile)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def overlap_ratio(self, bbox1: LabeledBoundingBox2D, bbox2: LabeledBoundingBox2D):
        rect1 = Rectangle(bbox1.min_x, bbox1.min_y, bbox1.max_x, bbox1.max_y)
        rect2 = Rectangle(bbox2.min_x, bbox2.min_y, bbox2.max_x, bbox2.max_y)
        return (rect1 & rect2).area() / rect1.area()

    def manhattan_distance(self, bbox1: LabeledBoundingBox2D, bbox2: LabeledBoundingBox2D):
        rect1 = Rectangle(bbox1.min_x, bbox1.min_y, bbox1.max_x, bbox1.max_y)
        rect2 = Rectangle(bbox2.min_x, bbox2.min_y, bbox2.max_x, bbox2.max_y)
        return rect1.dist(rect2)

    def _make_box_msg(self, raw: dict, offset: int) -> tuple[LabeledBoundingBox2D, str, str]:
        box_msg = LabeledBoundingBox2D()
        box_msg.probability = raw["conf"]
        cx, cy, w, h = raw["xywh"]
        box_msg.min_x = int(cx - w / 2)
        box_msg.max_x = int(cx + w / 2)
        box_msg.min_y = int(cy - h / 2)
        box_msg.max_y = int(cy + h / 2)

        label_name = raw["name"]
        class_name = f"{label_name}_{raw['cls']}"
        color_name = class_name.split("_")[0]

        color_label = -1
        if color_name in self.label_dict:
            color_label = self.label_dict[color_name]

        if self.use_color_names:
            box_msg.label = color_label
        else:
            box_msg.label = raw["cls"] + offset

        return box_msg, class_name, color_name

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def image_cb(self, msg: Image) -> None:
        # self.get_logger().info(f'IMAGE')
        # start_time = time.time()

        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_id = self._frame_id
        self._frame_id += 1

        # Send the same frame to every worker process
        for in_q in self._in_queues:
            try:
                in_q.put_nowait((frame_id, cv_image))
            except Exception:
                # Queue full — worker is still busy with the previous frame, skip
                self.get_logger().warn("Worker queue full, dropping frame for one model")

        # Collect exactly one result per worker for this frame_id
        per_model_results: dict[int, list] = {}
        while len(per_model_results) < self._num_models:
            result_frame_id, model_index, boxes, runtime = self._out_queue.get()
            if result_frame_id != frame_id:
                continue  # stale result from a dropped frame, discard
            # self.get_logger().info(f"[model {model_index}] yolo runtime: {runtime:.4f}s")
            per_model_results[model_index] = boxes

        # Annotate + bucket (main process only)
        annotator = Annotator(cv_image, 2, 2, "Arial.ttf", False)
        indicator_bboxes: list[LabeledBoundingBox2D] = []
        beacon_bboxes: list[LabeledBoundingBox2D] = []
        number_bboxes: list[LabeledBoundingBox2D] = []

        labeled_bounding_box_msgs = LabeledBoundingBox2DArray()
        labeled_bounding_box_msgs.header = msg.header

        for model_index in sorted(per_model_results):
            offset = self.label_offsets[model_index]
            for raw in per_model_results[model_index]:
                box_msg, class_name, color_name = self._make_box_msg(raw, offset)

                color = (128, 128, 128)
                text_color = (0, 0, 0)
                if color_name == "red":    color = (0, 0, 255)
                elif color_name == "green": color = (0, 255, 0)
                elif color_name == "blue":  color = (255, 0, 0)
                elif color_name == "yellow": color = (0, 230, 230)
                elif color_name == "black":
                    color = (0, 0, 0); text_color = (255, 255, 255)
                elif color_name == "white": color = (255, 255, 255)

                annotator.box_label(
                    (box_msg.min_x, box_msg.min_y, box_msg.max_x, box_msg.max_y),
                    class_name, color, text_color,
                )

                if not self.ignore_indicator_filters:
                    if box_msg.label in self.indicator_labels:
                        indicator_bboxes.append(box_msg)
                    elif box_msg.label in self.number_labels:
                        number_bboxes.append(box_msg)
                    else:
                        labeled_bounding_box_msgs.boxes.append(box_msg)
                    if box_msg.label == self.beacon_label:
                        beacon_bboxes.append(box_msg)
                else:
                    labeled_bounding_box_msgs.boxes.append(box_msg)

        # processing_start_time = time.time()

        if not self.ignore_indicator_filters:
            for indicator_box in indicator_bboxes:
                added = False
                for beacon_box in beacon_bboxes:
                    if self.overlap_ratio(indicator_box, beacon_box) > self.beacon_filter_ratio:
                        if self.indicator_to_beacon_bbox:
                            new_indicator = copy.deepcopy(beacon_box)
                            new_indicator.label = indicator_box.label
                            labeled_bounding_box_msgs.boxes.append(new_indicator)
                        else:
                            labeled_bounding_box_msgs.boxes.append(indicator_box)
                        added = True
                        break
                if not self.filter_beacon_indicators and not added:
                    labeled_bounding_box_msgs.boxes.append(indicator_box)

            for number_box in number_bboxes:
                new_number = copy.deepcopy(number_box)
                if self.match_indicators_banners:
                    min_dist = 10000000000
                    max_ratio = 0.0
                    for indicator_box in indicator_bboxes:
                        dist = self.manhattan_distance(number_box, indicator_box)
                        matched = False
                        if dist < min_dist:
                            min_dist = dist
                            if dist <= self.indicator_banner_px_dist:
                                new_number.label = self.label_dict[
                                    self.inv_label_dict[number_box.label] +
                                    ("_green" if indicator_box.label in self.green_indicator_labels else "_red")
                                ]
                                matched = True
                        if dist == 0:
                            ratio = self.overlap_ratio(number_box, indicator_box)
                            if ratio > max_ratio:
                                max_ratio = ratio
                                if not matched:
                                    new_number.label = self.label_dict[
                                        self.inv_label_dict[number_box.label] +
                                        ("_green" if indicator_box.label in self.green_indicator_labels else "_red")
                                    ]
                labeled_bounding_box_msgs.boxes.append(new_number)

        # self.get_logger().info(f"processing time: {time.time() - processing_start_time:.4f}s")

        self._pub.publish(labeled_bounding_box_msgs)
        annotated_image_msg = self.cv_bridge.cv2_to_imgmsg(annotator.result(), encoding="bgr8")
        self._image_pub.publish(annotated_image_msg)

        # self.get_logger().info(f"image yolo processing time: {time.time() - start_time:.4f}s")

    def destroy_node(self):
        # Send shutdown sentinel to every worker
        for in_q in self._in_queues:
            in_q.put(None)
        for p in self._workers:
            p.join(timeout=3)
        super().destroy_node()


def main():
    mp.set_start_method("spawn", force=True)  # required for CUDA in subprocesses
    rclpy.init()
    node = YOLOv11Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()