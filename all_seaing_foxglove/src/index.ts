// Arcturus Foxglove extension: lets Foxglove's built-in panels display our custom ROS messages
// (what all_seaing_rviz_plugins and RViz did), so Foxglove can replace RViz.
// Install: `npm install && npm run local-install` (desktop app), or `npm run package` and
// drag the .foxe file into Foxglove.

import { ExtensionContext } from "@foxglove/extension";

import {
  boxesToImageAnnotations,
  obstacleMapToSceneUpdate,
  planesToSceneUpdate,
  pointStampedToSceneUpdate,
} from "./converters";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "all_seaing_interfaces/msg/ObstacleMap",
    toSchemaName: "foxglove.SceneUpdate",
    converter: obstacleMapToSceneUpdate as (msg: unknown) => unknown,
  });
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "all_seaing_interfaces/msg/LabeledBoundingBox2DArray",
    toSchemaName: "foxglove.ImageAnnotations",
    converter: boxesToImageAnnotations as (msg: unknown) => unknown,
  });
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "all_seaing_interfaces/msg/LabeledObjectPlaneArray",
    toSchemaName: "foxglove.SceneUpdate",
    converter: planesToSceneUpdate as (msg: unknown) => unknown,
  });
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "geometry_msgs/msg/PointStamped",
    toSchemaName: "foxglove.SceneUpdate",
    converter: pointStampedToSceneUpdate as (msg: unknown) => unknown,
  });
}
