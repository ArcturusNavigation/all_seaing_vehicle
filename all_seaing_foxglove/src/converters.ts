// Message converters: turn all_seaing_interfaces (and other messages Foxglove doesn't draw)
// into Foxglove schemas, so the built-in 3D / Image panels can show them like RViz did.
// Conversion runs in Foxglove (on the laptop), so it costs nothing on the boat.

import {
  ImageAnnotations,
  LineType,
  PointsAnnotationType,
  SceneEntityDeletionType,
  SceneEntity,
  SceneUpdate,
} from "@foxglove/schemas";

import { labelColor, labelName, RGBA } from "./labels";

// ROS 2 message shapes (as deserialized by Foxglove: plain objects, snake_case fields)
type Time = { sec: number; nanosec: number };
type Header = { stamp: Time; frame_id: string };
type Point = { x: number; y: number; z: number };
type Quaternion = { x: number; y: number; z: number; w: number };
type PointStamped = { header: Header; point: Point };
type PolygonStamped = { header: Header; polygon: { points: Point[] } };

type Obstacle = {
  id: number;
  label: number;
  local_point: PointStamped;
  global_point: PointStamped;
  local_chull: PolygonStamped;
  global_chull: PolygonStamped;
  polygon_area: number;
};
type ObstacleMap = {
  header: Header;
  local_header: Header;
  ns: string;
  obstacles: Obstacle[];
  is_labeled: boolean;
};

type LabeledBoundingBox2D = {
  probability: number;
  min_x: number;
  min_y: number;
  max_x: number;
  max_y: number;
  label: number;
};
type LabeledBoundingBox2DArray = { header: Header; boxes: LabeledBoundingBox2D[] };

type LabeledObjectPlane = {
  id: number;
  label: number;
  normal_ctr: { position: Point; orientation: Quaternion };
  size: Point;
};
type LabeledObjectPlaneArray = {
  header: Header;
  objects: LabeledObjectPlane[];
  coplanar_merged: LabeledObjectPlane[];
  coplanar_indiv: LabeledObjectPlane[];
};

const IDENTITY = { x: 0, y: 0, z: 0, w: 1 };
const LIFETIME_ZERO = { sec: 0, nsec: 0 }; // entities live until replaced by the next message
const toTime = (t: Time) => ({ sec: t.sec, nsec: t.nanosec });

/**
 * ObstacleMap -> SceneUpdate, like the RViz ObstacleMap display (all_seaing_rviz_plugins):
 * per obstacle, the centroid, the convex hull (vertices + edges, at the centroid's height) and
 * the label text 1 m above. Uses the global_* fields and their frame, as the RViz plugin does.
 * Obstacles are colored by label color (unlabeled maps in green, as in RViz).
 */
export function obstacleMapToSceneUpdate(msg: ObstacleMap): SceneUpdate {
  const ns = msg.ns || "obstacles";
  const entities: SceneEntity[] = msg.obstacles.map((o) => {
    const frame = o.global_point.header.frame_id || msg.header.frame_id;
    const c = o.global_point.point;
    const color: RGBA = labelColor(o.label, { labeled: msg.is_labeled });
    const hull = o.global_chull.polygon.points.map((p) => ({ x: p.x, y: p.y, z: c.z }));
    return {
      timestamp: toTime(msg.header.stamp),
      frame_id: frame,
      id: `${ns}/${o.id}`,
      lifetime: LIFETIME_ZERO,
      frame_locked: false,
      metadata: [
        { key: "id", value: String(o.id) },
        { key: "label", value: msg.is_labeled ? `${o.label} (${labelName(o.label)})` : "unlabeled" },
        { key: "area", value: o.polygon_area.toFixed(3) },
      ],
      arrows: [],
      cubes: [],
      cylinders: [],
      lines:
        hull.length > 1
          ? [{ type: LineType.LINE_LOOP, pose: { position: { x: 0, y: 0, z: 0 }, orientation: IDENTITY },
               thickness: 0.04, scale_invariant: false, points: hull, color, colors: [], indices: [] }]
          : [],
      models: [],
      spheres: [
        { pose: { position: c, orientation: IDENTITY }, size: { x: 0.15, y: 0.15, z: 0.15 }, color },
        ...hull.map((p) => ({ pose: { position: p, orientation: IDENTITY }, size: { x: 0.05, y: 0.05, z: 0.05 }, color })),
      ],
      texts: msg.is_labeled
        ? [{ pose: { position: { x: c.x, y: c.y, z: c.z + 1.0 }, orientation: IDENTITY }, billboard: true,
             font_size: 14, scale_invariant: true, color, text: `${o.label} ${labelName(o.label)}` }]
        : [],
      triangles: [],
    };
  });
  // deletion of everything in this namespace first: obstacles that disappeared from the map vanish too
  return { deletions: [{ timestamp: toTime(msg.header.stamp), type: SceneEntityDeletionType.ALL, id: "" }], entities };
}

/** geometry_msgs/PointStamped -> SceneUpdate (a sphere), e.g. RViz's /clicked_point. */
export function pointStampedToSceneUpdate(msg: PointStamped): SceneUpdate {
  return {
    deletions: [],
    entities: [{
      timestamp: toTime(msg.header.stamp), frame_id: msg.header.frame_id, id: "point", lifetime: LIFETIME_ZERO,
      frame_locked: false, metadata: [], arrows: [], cubes: [], cylinders: [], lines: [], models: [],
      spheres: [{ pose: { position: msg.point, orientation: IDENTITY }, size: { x: 0.3, y: 0.3, z: 0.3 },
                  color: { r: 0.8, g: 0.16, b: 0.8, a: 1 } }],
      texts: [], triangles: [],
    }],
  };
}

/**
 * LabeledBoundingBox2DArray (YOLO detections) -> ImageAnnotations: draws the boxes with
 * label and confidence on the raw camera image in an Image panel (select the bounding box
 * topic as an annotation of the camera topic). The pixel coordinates are those of the image
 * YOLO ran on.
 */
export function boxesToImageAnnotations(msg: LabeledBoundingBox2DArray): ImageAnnotations {
  const ts = toTime(msg.header.stamp);
  return {
    circles: [],
    points: msg.boxes.map((b) => {
      const color = labelColor(b.label);
      return {
        timestamp: ts,
        type: PointsAnnotationType.LINE_LOOP,
        points: [
          { x: b.min_x, y: b.min_y }, { x: b.max_x, y: b.min_y },
          { x: b.max_x, y: b.max_y }, { x: b.min_x, y: b.max_y },
        ],
        outline_color: color,
        outline_colors: [],
        fill_color: { ...color, a: 0 },
        thickness: 3,
      };
    }),
    texts: msg.boxes.map((b) => ({
      timestamp: ts,
      position: { x: b.min_x, y: Math.max(0, b.min_y - 4) },
      text: `${labelName(b.label)} ${(b.probability * 100).toFixed(0)}%`,
      font_size: 16,
      text_color: labelColor(b.label),
      background_color: { r: 0, g: 0, b: 0, a: 0.6 },
    })),
  };
}

/**
 * LabeledObjectPlaneArray (RANSAC banner planes) -> SceneUpdate: one thin box per plane
 * (x = normal, size in y/z), with the label. Merged coplanar sets are drawn translucent.
 */
export function planesToSceneUpdate(msg: LabeledObjectPlaneArray): SceneUpdate {
  const plane = (p: LabeledObjectPlane, group: string, alpha: number): SceneEntity => {
    const color = { ...labelColor(p.label), a: alpha };
    return {
      timestamp: toTime(msg.header.stamp), frame_id: msg.header.frame_id, id: `${group}/${p.id}`,
      lifetime: LIFETIME_ZERO, frame_locked: false,
      metadata: [{ key: "label", value: `${p.label} (${labelName(p.label)})` }],
      arrows: [{ pose: p.normal_ctr, shaft_length: 0.6, shaft_diameter: 0.04, head_length: 0.15,
                 head_diameter: 0.1, color }],
      cubes: [{ pose: p.normal_ctr, size: { x: 0.03, y: Math.max(p.size.y, 0.05), z: Math.max(p.size.z, 0.05) }, color }],
      cylinders: [], lines: [], models: [], spheres: [],
      texts: [{ pose: { position: { ...p.normal_ctr.position, z: p.normal_ctr.position.z + 0.6 }, orientation: IDENTITY },
                billboard: true, font_size: 14, scale_invariant: true, color, text: labelName(p.label) }],
      triangles: [],
    };
  };
  return {
    deletions: [{ timestamp: toTime(msg.header.stamp), type: SceneEntityDeletionType.ALL, id: "" }],
    entities: [
      ...msg.objects.map((p) => plane(p, "plane", 0.9)),
      ...msg.coplanar_merged.map((p) => plane(p, "merged", 0.35)),
    ],
  };
}
