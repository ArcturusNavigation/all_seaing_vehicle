#!/usr/bin/env python3
"""
Convert RViz2 configs (.rviz) into equivalent Foxglove layouts (.json), so Foxglove can be
used the same way as RViz (with the all_seaing_foxglove extension installed for our custom
messages).

Mapping:
- 3D panel: one entry per RViz display, keeping its enabled flag (-> visible), color mode, point
  size/shape and alpha. Fixed Frame -> display frame, Grid / RobotModel (/robot_description) ->
  layers, orbit view -> camera, "Publish Point" tool -> publish clicked points on /clicked_point.
  Supported natively: PointCloud2, LaserScan, Marker(Array), Map (OccupancyGrid), TF, Grid,
  RobotModel, Pose. Via the extension: ObstacleMap, PointStamped, LabeledObjectPlaneArray.
- Image displays -> Image panels (with the YOLO boxes of that camera as annotations, if any).
- AerialMap (satellite tiles) -> Map panel following the GPS fix, with the offline tiles served
  by the control panel (http://localhost:8080/tiles/{z}/{x}/{y}.png).
- Every layout also gets a Diagnostics summary panel (StatusReporter / topic_monitor statuses).

Usage:
    python3 rviz_to_foxglove.py <file.rviz> [...] [-o OUT_DIR]   (default OUT_DIR: next to this script)
Then in Foxglove: Layouts > Import from file.
"""

import argparse
import json
import math
import os

import yaml

GPS_TOPIC = "/mavros/global_position/raw/fix"  # the RViz AerialMap used an outdated /gps/filtered
TILE_URL = "http://localhost:8080/tiles/{z}/{x}/{y}.png"
# YOLO boxes drawn over each camera's image (all_seaing_foxglove converts them to ImageAnnotations)
IMAGE_ANNOTATIONS = {
    "/zed/zed_node/rgb/image_rect_color": "/bounding_boxes",
    "/back_left_oak/rgb/image_rect": "/bounding_boxes/back_left",
    "/back_right_oak/rgb/image_rect": "/bounding_boxes/back_right",
}


def rgb_hex(rviz_color, default="#ffffff"):
    try:
        r, g, b = (int(c) for c in str(rviz_color).split(";"))
        return f"#{r:02x}{g:02x}{b:02x}"
    except ValueError:
        return default


def topic_of(display):
    t = display.get("Topic")
    if isinstance(t, dict):
        t = t.get("Value")
    if t and not t.startswith("/"):
        t = "/" + t
    return t


def flatten(displays):
    for d in displays or []:
        if "Displays" in d:  # display groups
            yield from flatten(d["Displays"])
        else:
            yield d


def pointcloud_settings(d, visible):
    s = {"visible": visible, "pointSize": float(d.get("Size (Pixels)", 3)),
         "pointShape": "square" if "Square" in str(d.get("Style", "")) else "circle"}
    transformer = d.get("Color Transformer", "")
    if transformer == "Intensity":
        s.update({"colorField": "intensity", "colorMode": "colormap", "colorMap": "turbo"})
    elif transformer == "RGB8":
        s.update({"colorField": "rgb", "colorMode": "rgb"})
    elif transformer in ("FlatColor", ""):
        s.update({"colorMode": "flat", "flatColor": rgb_hex(d.get("Color"))})
    else:  # AxisColor etc.
        s.update({"colorField": "z", "colorMode": "colormap", "colorMap": "turbo"})
    return s


def convert(rviz_path):
    with open(rviz_path) as f:
        vm = yaml.safe_load(f)["Visualization Manager"]
    name = os.path.splitext(os.path.basename(rviz_path))[0]
    fixed_frame = vm.get("Global Options", {}).get("Fixed Frame", "map")

    topics, layers, images, aerial = {}, {}, [], None
    for d in flatten(vm.get("Displays")):
        cls = d.get("Class", "").split("/")[-1]
        visible = bool(d.get("Enabled", False))
        topic = topic_of(d)
        if cls == "Grid":
            layers["grid"] = {"layerId": "foxglove.Grid", "visible": visible, "frameId": fixed_frame, "size": 100,
                              "divisions": 100, "lineWidth": 1, "color": rgb_hex(d.get("Color"), "#a0a0a4") + "80",
                              "position": [0, 0, 0], "rotation": [0, 0, 0], "order": 1, "label": "Grid"}
        elif cls == "RobotModel":
            layers["urdf"] = {"layerId": "foxglove.Urdf", "visible": visible, "sourceType": "topic",
                              "topic": "/robot_description", "label": "Robot model", "order": 2}
        elif cls == "Image" and topic:
            images.append((topic, visible))
        elif cls == "AerialMap":
            aerial = {"visible": visible, "zoom": int(d.get("Zoom", 16))}
        elif cls in ("PointCloud2", "LaserScan") and topic:
            # the same topic can appear twice in RViz (e.g. with different sizes): visible if any is
            prev = topics.get(topic)
            topics[topic] = pointcloud_settings(d, visible or bool(prev and prev["visible"]))
        elif cls == "Map" and topic:
            topics[topic] = {"visible": visible, "colorMode": "map", "alpha": float(d.get("Alpha", 0.7)),
                             "frameLocked": False}
        elif cls in ("Marker", "MarkerArray", "ObstacleMap", "PointStamped", "Pose", "PoseArray", "Path",
                     "Odometry") and topic:
            topics[topic] = {"visible": visible or bool(topics.get(topic, {}).get("visible"))}
        elif cls == "TF":
            pass  # transforms are always available in the 3D panel's Transforms section

    view = vm.get("Views", {}).get("Current", {})
    camera = {"distance": float(view.get("Distance", 30)), "perspective": True,
              "phi": math.degrees(float(view.get("Pitch", 0.8))), "thetaOffset": math.degrees(float(view.get("Yaw", 0))),
              "targetOffset": [0, 0, 0], "target": [0, 0, 0], "targetOrientation": [0, 0, 0, 1], "fovy": 45,
              "near": 0.01, "far": 5000}

    config = {}
    config["3D!main"] = {"cameraState": camera, "followMode": "follow-pose", "followTf": fixed_frame,
                         "scene": {"transforms": {"showLabel": False, "axisScale": 1}}, "transforms": {},
                         "topics": topics, "layers": layers,
                         "publish": {"type": "point", "pointTopic": "/clicked_point", "poseTopic": "/goal_pose",
                                     "poseEstimateTopic": "/initialpose"},
                         "imageMode": {}}
    image_ids = []
    for i, (topic, visible) in enumerate(images):
        pid = f"Image!img{i}"
        annotations = {}
        raw = topic
        if topic.startswith("/annotated_image"):  # already annotated by YOLO: no overlay needed
            pass
        elif topic in IMAGE_ANNOTATIONS:
            annotations[IMAGE_ANNOTATIONS[topic]] = {"visible": True}
        config[pid] = {"cameraState": {}, "followMode": "follow-pose", "scene": {}, "transforms": {}, "topics": {},
                       "layers": {}, "publish": {"type": "point"},
                       "imageMode": {"imageTopic": raw, "annotations": annotations}}
        image_ids.append(pid)
    config["DiagnosticSummary!diag"] = {"minLevel": 0, "pinnedIds": [], "hardwareIdFilter": "", "topicToRender": "/diagnostics",
                                        "sortByLevel": True}
    side = image_ids + ["DiagnosticSummary!diag"]
    if aerial:
        config["map!map"] = {"center": None, "customTileUrl": TILE_URL, "disabledTopics": [], "followTopic": GPS_TOPIC,
                             "layer": "custom", "topicColors": {}, "zoomLevel": aerial["zoom"], "maxNativeZoom": 18}
        side.insert(0, "map!map")

    def column(ids):
        node = ids[-1]
        for pid in reversed(ids[:-1]):
            node = {"first": pid, "second": node, "direction": "column"}
        return node

    layout = {"first": "3D!main", "second": column(side), "direction": "row", "splitPercentage": 65}
    return name, {"configById": config, "globalVariables": {}, "userNodes": {}, "playbackConfig": {"speed": 1},
                  "layout": layout}


def main():
    parser = argparse.ArgumentParser(description="convert RViz2 configs to Foxglove layouts")
    parser.add_argument("rviz", nargs="+")
    parser.add_argument("-o", "--out", default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()
    for path in args.rviz:
        name, layout = convert(path)
        out = os.path.join(args.out, f"{name}.json")
        with open(out, "w") as f:
            json.dump(layout, f, indent=2)
        print(f"{path} -> {out}")


if __name__ == "__main__":
    main()
