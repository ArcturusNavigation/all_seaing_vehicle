"""
LiDAR-camera fusion (bbox_project_pcloud) analyzer: what happened to the YOLO boxes of
each camera, i.e. how many became detections and why the others were rejected (#502).
Uses the "frame" (DEBUG), "box_rejected" and "frame_dropped" events.
"""

from collections import Counter, defaultdict

NAME = "fusion"
DESCRIPTION = "per-camera YOLO box funnel: detections vs rejection reasons (bbox_project_pcloud)"


def analyze(entries, args):
    boxes, detections, frames = Counter(), Counter(), Counter()
    reasons = defaultdict(Counter)  # camera -> reason -> count
    by_color = defaultdict(Counter)  # camera -> (color, reason) -> count
    dropped = defaultdict(Counter)  # camera -> frame drop reason -> count
    for e in entries:
        cam = e.get("camera") or e.get("node", "?")
        if e.get("event") == "frame":
            frames[cam] += 1
            boxes[cam] += e.get("n_boxes", 0)
            detections[cam] += e.get("n_detections", 0)
        elif e.get("event") in ("box_rejected", "box_skipped"):
            reason = e.get("reason", "?") if e["event"] == "box_rejected" else "skipped: " + e.get("reason", "?")
            reasons[cam][reason] += 1
            by_color[cam][(e.get("color", "?"), reason)] += 1
        elif e.get("event") == "frame_dropped":
            dropped[cam][e.get("reason", "?")] += 1

    if not frames and not reasons:
        return "No fusion events (is bbox_project_pcloud running with run_log_min_level DEBUG?)"
    lines = []
    for cam in sorted(set(frames) | set(reasons) | set(dropped)):
        n = boxes[cam]
        lines.append(f"[{cam}] frames: {frames[cam]}, boxes: {n}, detections: {detections[cam]}"
                     + (f" ({100 * detections[cam] / n:.0f}%)" if n else ""))
        for reason, c in dropped[cam].most_common():
            lines.append(f"    frames dropped ({reason}): {c}")
        for reason, c in reasons[cam].most_common():
            lines.append(f"    rejected {reason:28} {c:6d}" + (f" ({100 * c / n:.0f}%)" if n else ""))
        colors = sorted({col for col, _ in by_color[cam]})
        for col in colors:
            per = ", ".join(f"{r}={c}" for (cc, r), c in by_color[cam].most_common() if cc == col)
            lines.append(f"    {col:>8}: {per}")
    return "\n".join(lines)


def plots(entries, args, bucket=1.0):
    """
    Per camera, over time [boxes/s]: a stacked partition of the YOLO boxes into detections and each
    rejection reason (largest first), then the boxes skipped as non-buoys, with the total boxes in as
    an outline ("total" series): every box is in exactly one band, so the bands sum to the outline.
    """
    per_cam = defaultdict(lambda: defaultdict(Counter))  # camera -> series -> bucket -> count
    end = 0
    for e in entries:
        b = int(e["_rel"] // bucket)
        end = max(end, b + 1)
        if e.get("event") not in ("frame", "box_rejected", "box_skipped"):
            continue
        cam = e.get("camera") or e.get("node", "?")
        if e.get("event") == "frame":
            per_cam[cam]["boxes in"][b] += e.get("n_boxes", 0)
            per_cam[cam]["detections"][b] += e.get("n_detections", 0)
        elif e.get("event") == "box_rejected":
            per_cam[cam]["rejected: " + e.get("reason", "?")][b] += 1
        elif e.get("event") == "box_skipped":
            per_cam[cam]["skipped: " + e.get("reason", "?")][b] += 1
    xs = [i * bucket for i in range(end)]

    def series_of(name, c, **extra):
        return {"name": name, "x": xs, "y": [c.get(i, 0) / bucket for i in range(end)], **extra}

    out = []
    for cam, series in sorted(per_cam.items()):
        parts = [series_of("detections", series.get("detections", Counter()))]
        rejected = sorted((n for n in series if n.startswith("rejected: ")), key=lambda n: -sum(series[n].values()))
        parts += [series_of(n, series[n]) for n in rejected]
        parts += [series_of(n, series[n]) for n in sorted(series) if n.startswith("skipped: ")]  # on top of the stack
        parts.append(series_of("boxes in", series.get("boxes in", Counter()), total=True))
        out.append({"title": f"Fusion funnel: {cam}", "kind": "stacked", "x_label": "time [s]", "y_label": "boxes/s",
                    "series": parts})
    return out


def summary(entries, args):
    """Structured version of analyze() for rich display (control panel):
    {camera: {frames, boxes, detections, rejected: {reason: n}, skipped: {reason: n},
              by_color: {color: {reason (skipped ones prefixed "skipped: "): n}}, dropped: {reason: n}}}"""
    cams = defaultdict(lambda: {"frames": 0, "boxes": 0, "detections": 0, "rejected": Counter(), "skipped": Counter(),
                                "by_color": defaultdict(Counter), "dropped": Counter()})
    for e in entries:
        if e.get("event") not in ("frame", "box_rejected", "box_skipped", "frame_dropped"):
            continue  # other nodes' events (e.g. multicam_detection_merge) are not fusion data
        cam = e.get("camera") or e.get("node", "?")
        c = cams[cam]
        if e.get("event") == "frame":
            c["frames"] += 1
            c["boxes"] += e.get("n_boxes", 0)
            c["detections"] += e.get("n_detections", 0)
        elif e.get("event") == "box_rejected":
            c["rejected"][e.get("reason", "?")] += 1
            c["by_color"][e.get("color", "?")][e.get("reason", "?")] += 1
        elif e.get("event") == "box_skipped":
            c["skipped"][e.get("reason", "?")] += 1
            c["by_color"][e.get("color", "?")]["skipped: " + e.get("reason", "?")] += 1
        elif e.get("event") == "frame_dropped":
            c["dropped"][e.get("reason", "?")] += 1
    return {cam: {"frames": c["frames"], "boxes": c["boxes"], "detections": c["detections"],
                  "rejected": dict(c["rejected"]), "skipped": dict(c["skipped"]), "by_color": {k: dict(v) for k, v in c["by_color"].items()},
                  "dropped": dict(c["dropped"])} for cam, c in sorted(cams.items())}
