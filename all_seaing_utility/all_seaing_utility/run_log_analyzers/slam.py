"""
SLAM analyzer (object_tracking_map / factor_graph_slam): data association outcomes, landmark
creation/removal, and the map over time.

Uses the events these nodes log (see their object_track_map_publish):
- slam_update (DEBUG): per detection batch, the robot pose and every detection
  [label, x, y, result, landmark id, mahalanobis d2, nearest landmark index, its d2] with result
  0 matched, 1 new landmark, 2 unlabeled not added, 3 distance-gated -> new landmark
- slam_landmark_removed (INFO): id, label, position, reason (not_seen / duplicate)
- slam_map (DEBUG): after each update, pose (+ pose_cov, odom_pose), every landmark
  [id, label, x, y, cov_xx, cov_xy, cov_yy, dead] and (if tracked) every banner [id, label, x, y, theta, width]

analyze/summary/plots follow the other analyzers; view() returns map frames for the control
panel's map player.
"""

from collections import Counter, defaultdict

NAME = "slam"
DESCRIPTION = "SLAM: data association outcomes, landmarks added/removed, map size and pose correction over time"

RESULTS = {0: "matched", 1: "new landmark", 2: "unlabeled, not added", 3: "distance-gated -> new"}


def _t(e):
    return e.get("_time", e.get("wall", 0.0))


def summary(entries, args):
    """{node: {updates, detections, results: {name: n}, removed: {reason: n}, landmarks_last, landmarks_max,
    matched_d2_mean, correction_max_m (|SLAM pose - odometry pose|, if logged)}}"""
    out = defaultdict(lambda: {"updates": 0, "detections": 0, "results": Counter(), "removed": Counter(),
                               "landmarks_last": 0, "landmarks_max": 0, "d2": [], "correction_max_m": None})
    for e in entries:
        ev, s = e.get("event"), None
        if ev not in ("slam_update", "slam_landmark_removed", "slam_map"):
            continue
        s = out[e.get("node", "?")]
        if ev == "slam_update":
            s["updates"] += 1
            for d in e.get("detections", []):
                s["detections"] += 1
                s["results"][RESULTS.get(int(d[3]), str(d[3]))] += 1
                if int(d[3]) == 0 and d[5] >= 0:
                    s["d2"].append(d[5])
        elif ev == "slam_landmark_removed":
            s["removed"][e.get("reason", "?")] += 1
        else:
            n = len(e.get("landmarks", []))
            s["landmarks_last"], s["landmarks_max"] = n, max(s["landmarks_max"], n)
            if e.get("odom_pose") and e.get("pose"):
                c = ((e["pose"][0] - e["odom_pose"][0]) ** 2 + (e["pose"][1] - e["odom_pose"][1]) ** 2) ** 0.5
                s["correction_max_m"] = max(s["correction_max_m"] or 0.0, round(c, 3))
    res = {}
    for node, s in out.items():
        d2 = s.pop("d2")
        s["matched_d2_mean"] = round(sum(d2) / len(d2), 3) if d2 else None
        s["results"], s["removed"] = dict(s["results"]), dict(s["removed"])
        res[node] = s
    return res


def analyze(entries, args):
    s = summary(entries, args)
    if not s:
        return "no SLAM events (slam_update / slam_map) in this run"
    lines = []
    for node, v in s.items():
        lines.append(f"[{node}] updates {v['updates']}, detections {v['detections']}, "
                     f"landmarks now {v['landmarks_last']} (max {v['landmarks_max']})")
        for name, n in sorted(v["results"].items(), key=lambda x: -x[1]):
            lines.append(f"    {name:28} {n:6d} ({100 * n / max(1, v['detections']):.0f}%)")
        for reason, n in v["removed"].items():
            lines.append(f"    landmarks removed ({reason}): {n}")
        if v["matched_d2_mean"] is not None:
            lines.append(f"    mean Mahalanobis d2 of matches: {v['matched_d2_mean']}")
        if v["correction_max_m"] is not None:
            lines.append(f"    max |SLAM pose - odometry pose|: {v['correction_max_m']} m")
    return "\n".join(lines)


def plots(entries, args, bucket=1.0):
    """Per node: association outcomes over time (stacked, per second); landmark count and pose correction."""
    per = defaultdict(lambda: defaultdict(Counter))
    lm = defaultdict(list)  # node -> [(rel, n landmarks)]
    corr = defaultdict(list)  # node -> [(rel, |slam - odom|)]
    end = 0
    for e in entries:
        ev = e.get("event")
        if ev not in ("slam_update", "slam_map"):
            continue
        node, rel = e.get("node", "?"), e["_rel"]
        b = int(rel // bucket)
        end = max(end, b + 1)
        if ev == "slam_update":
            for d in e.get("detections", []):
                per[node][RESULTS.get(int(d[3]), str(d[3]))][b] += 1
        else:
            lm[node].append((round(rel, 2), len(e.get("landmarks", []))))
            if e.get("odom_pose") and e.get("pose"):
                corr[node].append((round(rel, 2), round(((e["pose"][0] - e["odom_pose"][0]) ** 2 +
                                                         (e["pose"][1] - e["odom_pose"][1]) ** 2) ** 0.5, 3)))
    xs = [i * bucket for i in range(end)]
    out = []
    for node in sorted(set(per) | set(lm)):
        series = [{"name": name, "x": xs, "y": [per[node][name].get(i, 0) / bucket for i in range(end)]}
                  for name in RESULTS.values() if name in per[node]]
        if series:
            out.append({"title": f"Data association: {node}", "kind": "stacked", "x_label": "time [s]",
                        "y_label": "detections/s", "series": series})
        line = [{"name": "landmarks", "x": [p[0] for p in lm[node]], "y": [p[1] for p in lm[node]]}]
        if corr[node]:
            line.append({"name": "|SLAM - odometry| [m]", "x": [p[0] for p in corr[node]], "y": [p[1] for p in corr[node]]})
        out.append({"title": f"Map size and pose correction: {node}", "kind": "line", "x_label": "time [s]",
                    "y_label": "landmarks / m", "series": line})
    return out


def view(entries, args, max_frames=1500):
    """Map frames for the control panel's map player: one per slam_map (subsampled to max_frames),
    each with the matching slam_update's detections and the landmarks removed since the previous frame.
    Times are run times [s] (`_rel`) and absolute (`_time`)."""
    maps = [e for e in entries if e.get("event") == "slam_map"]
    if not maps:
        return {"domain": "slam", "frames": []}
    nodes = Counter(e.get("node") for e in maps)
    node = nodes.most_common(1)[0][0]  # the SLAM node of this run
    maps = [e for e in maps if e.get("node") == node]
    # each slam_map follows the slam_update that produced it (log order; labeled and unlabeled batches
    # can share a stamp, so stamps can't pair them)
    update_of, last = {}, None
    for e in entries:
        if e.get("node") != node:
            continue
        if e.get("event") == "slam_update":
            last = e
        elif e.get("event") == "slam_map":
            update_of[id(e)] = last
            last = None
    removed = [e for e in entries if e.get("event") == "slam_landmark_removed" and e.get("node") == node]
    step = max(1, len(maps) // max_frames)
    frames, ri, trail = [], 0, []
    for i, m in enumerate(maps):
        trail.append(m.get("pose", [0, 0, 0])[:2])
        if i % step and i != len(maps) - 1:
            continue
        gone = []
        while ri < len(removed) and _t(removed[ri]) <= _t(m):
            r = removed[ri]
            gone.append({"id": r.get("id"), "label": r.get("label"), "position": r.get("position"), "reason": r.get("reason")})
            ri += 1
        u = update_of.get(id(m)) or {}
        frames.append({"t": round(m["_rel"], 3), "time": _t(m), "pose": m.get("pose"), "pose_cov": m.get("pose_cov"),
                       "odom_pose": m.get("odom_pose"), "landmarks": m.get("landmarks", []),
                       "banners": m.get("banners", []),  # [id, label, x, y, theta (normal), width]
                       "detections": u.get("detections", []), "removed": gone,
                       "update": {k: u.get(k) for k in ("algorithm", "threshold", "distance_gate_m", "labeled")} if u else None})
    return {"domain": "slam", "node": node, "frame_id": maps[-1].get("frame"),
            "trail": [[round(x, 2), round(y, 2)] for x, y in trail[:: max(1, len(trail) // 5000)]],
            "trail_t": [round(m["_rel"], 2) for m in maps[:: max(1, len(trail) // 5000)]],
            "results": RESULTS, "frames": frames}
