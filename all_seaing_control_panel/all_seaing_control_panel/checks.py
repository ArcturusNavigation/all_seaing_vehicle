"""
Pre-run checks (issue #327). Each check returns
    {"id", "title", "status": ok|warn|fail|unknown, "message", "details" (optional), "fix" (optional action id)}
Checks are configured in config/checks.yaml (expected devices, topics, TF pairs, thresholds).
Most run a shell command on the boat target. The heading and TF checks use the panel's ROS
monitor, and the topic checks use topic_monitor.py's /diagnostics.
"""

import shlex
import time


def _r(check_id, title, status, message, details=None, fix=None):
    return {"id": check_id, "title": title, "status": status, "message": message,
            "details": details or {}, "fix": fix}


# What each check verifies (shown on the Checks page)
CHECK_DESCRIPTIONS = {
    "connection": "Commands can be run on the target over SSH.",
    "clock": "The boat's clock matches the laptop's. An unset clock (e.g. 1970 after a reboot without network) breaks message timestamps and run-log dates.",
    "disk": "Free space on the root disk and the NVMe (bags, logs). Recording or logging fails when full.",
    "serial_ports": "The Pixhawk and central hub USB devices exist and are readable/writable (otherwise run the chmod fix or set up udev rules).",
    "mavros": "MAVROS is running and connected to the Pixhawk (/mavros/state).",
    "mavros_rate": "The Pixhawk streams IMU/odometry at >= 25 Hz. It resets to a low rate on restart; slow odometry breaks SLAM.",
    "central_hub": "The central hub (EE board) answers: needed for the E-stop status, servos and fans.",
    "sensors": "All sensor and pipeline topics are published at their expected rates (from the topic monitor), and sensors come from the right source: drivers on the live boat, the bag player in replay.",
    "lidar_time": "Sensor message timestamps are within 1 s of the clock (time sync between sensors and the computer).",
    "tf": "The transforms between the map, the boat and every sensor exist (needed for fusion and navigation).",
    "heading": "The compass/IMU heading points where the boat actually moves (GPS course), and forward motion gives positive body x velocity. Drive straight at > 0.5 m/s to run it.",
    "diagnostics": "No component reports ERROR/STALE (or WARN) on /diagnostics.",
    "git": "Which branch/commit the boat is running, and whether it has uncommitted changes.",
}

# Checks that don't apply in some modes: check id -> {mode: reason shown instead of running it}
MODE_SKIPS = {
    "serial_ports": {"replay": "no hardware: data comes from the rosbag", "sim": "no hardware in simulation"},
    "mavros": {"replay": "MAVROS topics come from the rosbag", "sim": "no Pixhawk in simulation"},
    "central_hub": {"replay": "no EE board: data comes from the rosbag", "sim": "no EE board in simulation"},
    "mavros_rate": {"sim": "no MAVROS in simulation"},
    "clock": {"replay": "nodes use the bag's /clock (use_sim_time)", "sim": "nodes use the simulation /clock"},
}


class Checks:
    def __init__(self, cfg, targets, monitor):
        self.cfg = cfg
        self.targets = targets
        self.monitor = monitor
        self.order = [
            ("connection", "Connection to the boat computer", self.connection),
            ("clock", "Clock sync (boat vs laptop)", self.clock),
            ("disk", "Disk space", self.disk),
            ("serial_ports", "Serial devices present & permissions", self.serial_ports),
            ("mavros", "MAVROS connected to the Pixhawk", self.mavros),
            ("mavros_rate", "MAVROS stream rate", self.mavros_rate),
            ("central_hub", "Central hub / E-stop comms", self.central_hub),
            ("sensors", "Sensor & pipeline topics", self.sensors),
            ("lidar_time", "LiDAR / camera timestamps vs clock", self.sensor_time),
            ("tf", "TF tree", self.tf),
            ("heading", "IMU/compass heading vs GPS course", self.heading),
            ("diagnostics", "Diagnostics errors", self.diagnostics),
            ("git", "Code version on the boat", self.git),
        ]

    def list(self):
        return [{"id": i, "title": t, "description": CHECK_DESCRIPTIONS.get(i, "")} for i, t, _ in self.order]

    def detect_mode(self, target_name):
        """
        'replay' if a rosbag player is running, 'sim' if Gazebo / ros_gz nodes are, else 'live'.
        Returns (mode, nodes).
        """
        rc, out, _ = self.targets[target_name].run("timeout 8 ros2 node list 2>/dev/null", timeout=15)
        nodes = [n.strip() for n in out.splitlines() if n.strip()]
        if any("rosbag2_player" in n for n in nodes):
            return "replay", nodes
        if any(k in n for n in nodes for k in ("ros_gz", "gz_", "gazebo", "/sim_")):
            return "sim", nodes
        return "live", nodes

    def run(self, check_id, target_name, mode="auto"):
        if mode == "auto":
            mode, _ = self.detect_mode(target_name)
        for i, title, fn in self.order:
            if i == check_id:
                skip = MODE_SKIPS.get(i, {}).get(mode)
                if skip:
                    return _r(i, title, "skipped", f"{skip} ({mode} mode)")
                try:
                    res = fn(self.targets[target_name], mode)
                except Exception as e:
                    res = ("fail", f"check crashed: {e}", None, None)
                status, message, details, fix = (list(res) + [None, None])[:4]
                return _r(i, title, status, message, details, fix)
        raise KeyError(check_id)

    # ------------------------------------------------------------------ helpers

    def _topic_status(self):
        """topic_monitor statuses by topic name, or {} if topic_monitor isn't running."""
        out = {}
        for d in self.monitor.diagnostics_snapshot():
            if d["name"].startswith("topic_monitor/"):
                out["/" + d["name"][len("topic_monitor/"):]] = d
        return out

    # ------------------------------------------------------------------ checks

    def connection(self, t, mode="live"):
        rc, out, err = t.run("hostname; uptime -p", timeout=10)
        if rc != 0:
            return "fail", f"cannot run commands on {t.label}: {err.strip()}"
        return "ok", out.strip().replace("\n", ", ")

    def clock(self, t, mode="live"):
        before = time.time()
        rc, out, err = t.run("date +%s.%N", timeout=10)
        after = time.time()
        if rc != 0:
            return "fail", err.strip()
        diff = float(out.strip()) - (before + after) / 2
        details = {"boat_minus_laptop_s": round(diff, 3)}
        if abs(diff) > 3600 * 24:
            return "fail", f"boat clock is off by {diff / 86400:.0f} days (no NTP?), stamps & logs will be wrong", details
        if abs(diff) > 0.5:
            return "warn", f"boat clock differs from the laptop by {diff:.2f} s", details
        return "ok", f"within {abs(diff) * 1000:.0f} ms", details

    def disk(self, t, mode="live"):
        paths = self.cfg.get("disk_paths", ["/"])
        rc, out, err = t.run("df -P " + " ".join(shlex.quote(p) for p in paths) + " | tail -n +2", timeout=10)
        if rc != 0:
            return "fail", err.strip()
        worst, details = 0, {}
        for line in out.strip().splitlines():
            f = line.split()
            used = int(f[4].rstrip("%"))
            details[f[5]] = f"{used}% used, {int(f[3]) / 1e6:.1f} GB free"
            worst = max(worst, used)
        status = "fail" if worst >= 98 else "warn" if worst >= 90 else "ok"
        return status, "; ".join(f"{k}: {v}" for k, v in details.items()), details

    def serial_ports(self, t, mode="live"):
        devices = self.cfg.get("serial_devices", [])
        script = "for d in " + " ".join(shlex.quote(d["path"]) for d in devices) + \
                 '; do if [ -e "$d" ]; then if [ -r "$d" ] && [ -w "$d" ]; then echo "$d ok"; ' \
                 'else echo "$d noperm"; fi; else echo "$d missing"; fi; done'
        rc, out, err = t.run(script, timeout=10)
        results = dict(line.split(" ", 1) for line in out.strip().splitlines() if " " in line)
        details, missing, noperm = {}, [], []
        for d in devices:
            r = results.get(d["path"], "missing")
            details[d["name"]] = f"{d['path']}: {r}"
            (missing if r == "missing" else noperm if r == "noperm" else []).append(d["name"])
        if missing:
            return "fail", "missing: " + ", ".join(missing), details
        if noperm:
            return "fail", "no read/write permission: " + ", ".join(noperm), details, "allow_ports"
        return "ok", "all devices present and accessible", details

    def mavros(self, t, mode="live"):
        rc, out, err = t.run("timeout 6 ros2 topic echo --once --field connected /mavros/state", timeout=12)
        if rc != 0 or not out.strip():
            return "fail", "no /mavros/state (is vehicle.launch.py / mavros running?)"
        connected = out.strip().splitlines()[0].strip().lower() == "true"
        return ("ok", "connected to the FCU") if connected else ("fail", "MAVROS running but not connected to the Pixhawk")

    def mavros_rate(self, t, mode="live"):
        topic = self.cfg.get("mavros_rate_topic", "/mavros/imu/data")
        expected = float(self.cfg.get("mavros_min_rate", 25.0))
        status = self._topic_status().get(topic)
        if status is not None:
            rate = float(status["values"].get("messages/rate", 0))
            source = "topic_monitor"
        else:
            rc, out, _ = t.run(f"timeout 6 ros2 topic hz {shlex.quote(topic)} -w 30 2>/dev/null | grep 'average rate' | tail -1",
                               timeout=12)
            if not out.strip():
                return "fail", f"no messages on {topic}", None, "mavros_rate_30"
            rate = float(out.split(":")[1])
            source = "ros2 topic hz"
        details = {"topic": topic, "rate_hz": round(rate, 1), "source": source}
        if rate < expected:
            if mode == "replay":
                return "warn", f"{topic} at {rate:.1f} Hz in the bag (recorded with a low MAVROS rate)", details
            return "fail", f"{topic} at {rate:.1f} Hz (< {expected:g} Hz): set the MAVROS rate", details, "mavros_rate_30"
        return "ok", f"{topic} at {rate:.1f} Hz", details

    def central_hub(self, t, mode="live"):
        rc, out, err = t.run("timeout 6 ros2 service call /get_estop_status all_seaing_interfaces/srv/GetEstopStatus",
                             timeout=12)
        if rc != 0 or "response" not in out:
            return "fail", "get_estop_status service not responding (central_hub_ros running? port right?)"
        resp = out.split("response:")[-1].strip().replace("\n", " ")
        return "ok", resp[:200]

    def sensors(self, t, mode="live"):
        statuses = self._topic_status()
        if not statuses:
            return "unknown", "topic_monitor isn't running (start it in Run > Monitoring)", None, "topic_monitor"
        bad, details = [], {}
        sensor_topics = self.cfg.get("sensor_topics", [])
        for topic in sensor_topics + self.cfg.get("pipeline_topics", []):
            s = statuses.get(topic)
            if s is None:
                details[topic] = "not monitored (add it to topic_monitor.yaml)"
                bad.append(topic)
                continue
            details[topic] = f"{s['level']}: {s['message']}"
            if s["level"] != "OK":
                bad.append(topic)
        # Who publishes each sensor topic: in replay all must come from the bag player, live none of them
        # (pipeline topics are always computed live, so they aren't checked for their source)
        topics = sensor_topics
        # only the publishers section of `ros2 topic info -v` (it lists subscribers after "Subscription count")
        script = "; ".join(f"echo '@@{tp}'; timeout 5 ros2 topic info -v {shlex.quote(tp)} 2>/dev/null | "
                           "awk '/^Subscription count/{exit} /^Node name:/{print}'" for tp in topics)
        rc, out, _ = t.run(script, timeout=10 + 6 * len(topics))
        mixed = []
        for chunk in out.split("@@")[1:]:
            tp, _, rest = chunk.partition("\n")
            pubs = [l.split(":", 1)[1].strip() for l in rest.splitlines() if ":" in l]
            if tp in details:
                details[tp] += f" | published by: {', '.join(pubs) or 'nobody'}"
            from_bag = any("rosbag2_player" in p for p in pubs)
            others = [p for p in pubs if "rosbag2_player" not in p and not p.startswith("_")]
            if (mode == "replay" and others) or (mode == "live" and from_bag):
                mixed.append(tp)
        if mixed:
            what = "also published by live nodes (bag + live data mixed)" if mode == "replay" \
                else "published by a rosbag player on the live boat"
            return "fail", f"{', '.join(mixed)}: {what}", details
        if bad:
            return "fail", f"{len(bad)} topic(s) not OK: " + ", ".join(bad), details
        src = "from the rosbag" if mode == "replay" else "from live nodes" if mode == "live" else "from the simulation"
        return "ok", f"all {len(details)} topics OK ({src})", details

    def sensor_time(self, t, mode="live"):
        statuses = self._topic_status()
        if not statuses:
            return "unknown", "topic_monitor isn't running", None, "topic_monitor"
        max_age = float(self.cfg.get("max_stamp_age_ms", 1000.0))
        details, bad = {}, []
        for topic in self.cfg.get("stamped_topics", []):
            s = statuses.get(topic)
            age = s["values"].get("stamp_age_ms") if s else None
            if age is None:
                details[topic] = "no data"
                continue
            details[topic] = f"{float(age):.0f} ms"
            if abs(float(age)) > max_age:
                bad.append(topic)
        if not details or all(v == "no data" for v in details.values()):
            return "unknown", "no stamped sensor data yet", details
        if bad:
            return "fail", "timestamps far from the clock (time sync?): " + ", ".join(bad), details
        return "ok", "sensor timestamps consistent with the clock", details

    def tf(self, t, mode="live"):
        details, missing = {}, []
        for pair in self.cfg.get("tf_pairs", []):
            ok = self.monitor.can_transform(pair[0], pair[1])
            details[f"{pair[0]} <- {pair[1]}"] = "ok" if ok else "missing"
            if not ok:
                missing.append(f"{pair[0]}<-{pair[1]}")
        if missing:
            return "fail", "missing: " + ", ".join(missing), details
        return "ok", f"all {len(details)} transforms available", details

    def heading(self, t, mode="live"):
        r = self.monitor.heading_check()
        return r["status"], r["message"], r.get("details")

    def diagnostics(self, t, mode="live"):
        diags = self.monitor.diagnostics_snapshot()
        if not diags:
            return "unknown", "no /diagnostics received"
        bad = [d for d in diags if d["level"] in ("ERROR", "STALE")]
        warn = [d for d in diags if d["level"] == "WARN"]
        details = {d["name"]: f"{d['level']}: {d['message']}" for d in bad + warn}
        if bad:
            return "fail", f"{len(bad)} component(s) ERROR/STALE", details
        if warn:
            return "warn", f"{len(warn)} component(s) WARN", details
        return "ok", f"{len(diags)} components OK"

    def git(self, t, mode="live"):
        repo = t.path("repo")
        rc, out, err = t.run(f"cd {shlex.quote(repo)} && git rev-parse --abbrev-ref HEAD && git log -1 --format='%h %s' "
                             f"&& git status --porcelain | wc -l", timeout=10)
        if rc != 0:
            return "warn", err.strip()
        branch, commit, dirty = out.strip().splitlines()
        details = {"branch": branch, "commit": commit, "uncommitted_files": int(dirty)}
        status = "warn" if int(dirty) else "ok"
        return status, f"{branch} @ {commit}" + (f" ({dirty} uncommitted file(s))" if int(dirty) else ""), details
