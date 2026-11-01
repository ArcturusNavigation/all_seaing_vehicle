"""
Tasks analyzer (run_tasks, task servers, navigation, controller, control mux): what the autonomy
decided and did, with coordinates.

Events (see ActionServerBase / TaskServerBase, run_tasks.py, controller_server.py,
navigation_server_continuous.py, control_mux.py); `pose` = robot [x, y, heading] in `frame`:
- run_tasks: task_attempt, task_goal_response, task_result, task_cancel, task_skipped_no_gate,
  slam_restart, robot_state
- task servers: task_goal, task_phase, task_state_change, task_state (DEBUG snapshot: state,
  waypoint, gate {left, right}, gate_waypoint, obstacles [[x, y, label]]), task_finished,
  nav_goal (kind follow_path / waypoint / continuous, target), nav_goal_updated / _resent /
  _rejected / _result, nav_cancel, gate_found / gate_not_found / gate_fake, report (competition)
- every action server: process_start / process_end (message = why it ended)
- controller_server: waypoint_goal, control (DEBUG: target, error_robot, pid_vel, avoid_vel, cmd)
- navigation_server_continuous: follow_path_goal, continuous_waypoint_update, waypoint_crossed,
  nav_control (DEBUG: goal, carrot, distance, desired_heading, avoid_vel, cmd), nav_aborted_no_map
- control_mux: mux_source (switches), cmd_vel (DEBUG)
"""

from collections import Counter, defaultdict

NAME = "tasks"
DESCRIPTION = "tasks: attempts/results, state machine timeline, navigation goals, gates, competition reports"

# events shown in the map player's event list (decisions, not periodic snapshots)
DECISIONS = {"task_attempt", "task_goal_response", "task_result", "task_cancel", "task_skipped_no_gate",
             "slam_restart", "robot_state", "task_goal", "task_phase", "task_state_change", "task_finished",
             "nav_goal", "nav_goal_updated", "nav_goal_resent", "nav_goal_rejected", "nav_goal_result", "nav_cancel",
             "gate_found", "gate_not_found", "gate_fake", "report", "process_start", "process_end",
             "waypoint_goal", "follow_path_goal", "continuous_waypoint_update", "waypoint_crossed",
             "nav_aborted_no_map", "mux_source"}
SNAPSHOTS = {"task_state", "control", "nav_control"}


def _t(e):
    return e.get("_time", e.get("wall", 0.0))


def state_segments(entries):
    """[(task, state, start rel, end rel)] from task_state_change (and task_finished closing a task)."""
    segs, open_ = [], {}
    for e in entries:
        ev, task = e.get("event"), e.get("task")
        if ev == "task_state_change":
            if task in open_:
                segs.append((task, open_[task][0], open_[task][1], e["_rel"]))
            open_[task] = (e.get("state") or "(none)", e["_rel"])
        elif ev == "task_finished" and task in open_:
            segs.append((task, open_[task][0], open_[task][1], e["_rel"]))
            del open_[task]
    end = max((e["_rel"] for e in entries), default=0)
    segs += [(task, st, t0, end) for task, (st, t0) in open_.items()]
    return segs


def summary(entries, args):
    s = {"attempts": [], "results": Counter(), "states": [], "nav_goals": Counter(), "nav_results": Counter(),
         "gates": Counter(), "reports": Counter(), "robot_states": [], "mux_switches": 0, "warnings": 0}
    for e in entries:
        ev = e.get("event")
        if ev == "task_attempt":
            s["attempts"].append({"t": round(e["_rel"], 2), "task": e.get("task"), "type": e.get("action_type")})
        elif ev == "task_result":
            s["results"][f"{e.get('task')}: {e.get('outcome')}"] += 1
        elif ev == "nav_goal":
            s["nav_goals"][e.get("kind", "?")] += 1
        elif ev == "nav_goal_result":
            s["nav_results"][e.get("result", "?")] += 1
        elif ev in ("gate_found", "gate_not_found", "gate_fake"):
            s["gates"][ev] += 1
        elif ev == "report":
            s["reports"][e.get("type", "?")] += 1
        elif ev == "robot_state":
            s["robot_states"].append({"t": round(e["_rel"], 2), "state": e.get("state")})
        elif ev == "mux_source":
            s["mux_switches"] += 1
        if e.get("level") in ("WARN", "ERROR") and ev in DECISIONS:
            s["warnings"] += 1
    s["states"] = [{"task": t, "state": st, "start": round(a, 2), "end": round(b, 2)} for t, st, a, b in state_segments(entries)]
    for k in ("results", "nav_goals", "nav_results", "gates", "reports"):
        s[k] = dict(s[k])
    return s


def analyze(entries, args):
    s = summary(entries, args)
    lines = [f"task attempts: {len(s['attempts'])}"]
    lines += [f"    {a['t']:8.1f}s  {a['task']} ({a['type']})" for a in s["attempts"]]
    lines.append("results: " + (", ".join(f"{k} x{n}" for k, n in s["results"].items()) or "none"))
    lines.append("states:")
    lines += [f"    {x['start']:8.1f}-{x['end']:8.1f}s  {x['task']}: {x['state']}" for x in s["states"]]
    lines.append(f"navigation goals: {s['nav_goals']}  results: {s['nav_results']}")
    lines.append(f"gates: {s['gates']}  reports: {s['reports']}  mux switches: {s['mux_switches']}")
    return "\n".join(lines)


def plots(entries, args):
    out = []
    segs = state_segments(entries)
    if segs:
        out.append({"title": "Task states", "kind": "timeline", "x_label": "time [s]",
                    "segments": [{"label": f"{t}: {st}", "start": a, "end": b} for t, st, a, b in segs]})
    dist = [(e["_rel"], e["distance"]) for e in entries if e.get("event") == "nav_control" and "distance" in e]
    if dist:
        out.append({"title": "Distance to the navigation goal", "kind": "line", "x_label": "time [s]", "y_label": "m",
                    "series": [{"name": "distance", "x": [d[0] for d in dist], "y": [d[1] for d in dist]}]})
    cmds = [(e["_rel"], e["cmd"]) for e in entries if e.get("event") == "cmd_vel" and e.get("cmd")]
    if cmds:
        out.append({"title": "Velocity command (control mux -> cmd_vel)", "kind": "line", "x_label": "time [s]",
                    "y_label": "m/s, rad/s", "series": [
                        {"name": n, "x": [c[0] for c in cmds], "y": [c[1][i] for c in cmds]}
                        for i, n in enumerate(("forward", "lateral", "yaw rate"))]})
    return out


def view(entries, args, max_frames=1500):
    """Map frames for the map player: at each snapshot time (task_state, control, nav_control), the
    latest of each (what the task knows/targets, controller and navigation steering), the robot trail,
    and the decision events up to then (listed and drawn: goals, gates, reports, ...)."""
    snaps = [e for e in entries if e.get("event") in SNAPSHOTS]
    decisions = [e for e in entries if e.get("event") in DECISIONS]
    if not snaps and not decisions:
        return {"domain": "tasks", "frames": []}
    timeline = sorted(snaps + decisions, key=lambda e: e["_rel"])
    step = max(1, len(timeline) // max_frames)
    latest, frames, trail = {}, [], []
    clean = lambda e: {k: v for k, v in e.items() if not k.startswith("_") and k not in ("t", "wall")}
    for i, e in enumerate(timeline):
        latest[e["event"] if e["event"] in SNAPSHOTS else "decision"] = e
        if e.get("pose") and e["event"] in SNAPSHOTS:
            trail.append([round(e["_rel"], 2)] + e["pose"][:2])
        if i % step and i != len(timeline) - 1:
            continue
        pose = next((latest[k]["pose"] for k in ("control", "nav_control", "task_state") if k in latest and latest[k].get("pose")), None)
        frames.append({"t": round(e["_rel"], 3), "time": _t(e), "pose": pose,
                       "task_state": clean(latest["task_state"]) if "task_state" in latest else None,
                       "control": clean(latest["control"]) if "control" in latest else None,
                       "nav_control": clean(latest["nav_control"]) if "nav_control" in latest else None})
    # decision list: without the periodic heartbeat reports, and mux switch bursts (e.g. teleop and
    # autonomy alternating every few hundred ms) collapsed into one entry with a count
    events, last_mux = [], None
    for e in decisions:
        if e["event"] == "report" and e.get("type") == "Heartbeat":
            continue
        if e["event"] == "mux_source":
            if last_mux is not None and e["_rel"] - last_mux["_t"] < 1.0:
                last_mux["switches"] = last_mux.get("switches", 1) + 1
                last_mux["_t"] = e["_rel"]
                continue
            last_mux = {"t": round(e["_rel"], 3), "_t": e["_rel"], **clean(e)}
            events.append(last_mux)
            continue
        events.append({"t": round(e["_rel"], 3), **clean(e)})
    for ev in events:
        ev.pop("_t", None)
    return {"domain": "tasks", "frame_id": next((e.get("frame") for e in timeline if e.get("frame")), None),
            "trail": trail[:: max(1, len(trail) // 5000)], "events": events,
            "states": [{"task": t, "state": st, "start": round(a, 2), "end": round(b, 2)} for t, st, a, b in state_segments(entries)],
            "frames": frames}
