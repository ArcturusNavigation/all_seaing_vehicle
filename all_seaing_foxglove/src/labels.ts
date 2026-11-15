// Label ids -> names, copied from all_seaing_bringup/config/perception/all_label_mappings.yaml.
// Keep in sync when the YOLO label mapping changes (unknown ids are shown as numbers).
export const LABEL_NAMES: Record<number, string> = {
  0: "black_buoy", 1: "black_circle", 2: "black_cross", 3: "black_triangle", 4: "blue_buoy",
  5: "blue_circle", 6: "blue_cross", 7: "blue_racquet_ball", 8: "blue_triangle", 9: "dock",
  10: "green_buoy", 11: "green_circle", 12: "green_cross", 13: "green_pole_buoy", 14: "green_triangle",
  15: "red_buoy", 16: "red_circle", 17: "red_cross", 18: "red_pole_buoy", 19: "red_racquet_ball",
  20: "red_square", 21: "red_triangle", 22: "yellow_buoy", 23: "yellow_racquet_ball", 24: "number_1",
  25: "number_2", 26: "number_3", 27: "beacon", 28: "green_indicator", 29: "red_indicator",
  30: "number_1_green", 31: "number_2_green", 32: "number_3_green", 33: "number_1_red",
  34: "number_2_red", 35: "number_3_red",
};

export type RGBA = { r: number; g: number; b: number; a: number };

const COLORS: Record<string, RGBA> = {
  red: { r: 0.9, g: 0.15, b: 0.15, a: 1 },
  green: { r: 0.1, g: 0.8, b: 0.25, a: 1 },
  yellow: { r: 0.95, g: 0.85, b: 0.1, a: 1 },
  blue: { r: 0.2, g: 0.45, b: 1.0, a: 1 },
  black: { r: 0.25, g: 0.25, b: 0.25, a: 1 },
  other: { r: 0.85, g: 0.4, b: 0.9, a: 1 },
  unlabeled: { r: 0.0, g: 1.0, b: 0.0, a: 1 }, // same green as the RViz ObstacleMap display
};

export function labelName(label: number): string {
  return LABEL_NAMES[label] ?? String(label);
}

/** Display color from the color word in the label name (red_buoy -> red, ...). */
export function labelColor(label: number, { labeled = true }: { labeled?: boolean } = {}): RGBA {
  if (!labeled) {
    return COLORS.unlabeled!;
  }
  const name = LABEL_NAMES[label] ?? "";
  for (const c of ["red", "green", "yellow", "blue", "black"]) {
    if (name.includes(c)) {
      return COLORS[c]!;
    }
  }
  return COLORS.other!;
}
