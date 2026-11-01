"""
Rosbag recording from the Run tab: presets (config/rosbag_presets.yaml, imported from the scripts
in all_seaing_bringup/rosbag_record/), started as panel processes on a target, and their storage.

A preset {id, title, prefix, all_topics, topics, source} records with
    ros2 bag record -o <target bags path>/<prefix>_<MM_DD__HH_MM_SS> <topics...>   (or --all)
(the scripts' folder naming, plus seconds so two recordings in the same minute don't collide).

Recordings are ordinary processes (processes.py) with action "rosbag" and args {preset, bag_dir,
linked_to}: stopping sends SIGINT, so ros2 bag closes the bag cleanly. `linked_to` is the process
the recording was started with (Run tab "record rosbag" option); it is stopped together with it.

Presets are edited from the panel, which rewrites the YAML file (resolved through the install
symlink, so the change lands in the repo).
"""

import os
import re
import shlex
import threading
import time

import yaml

HEADER_END = "presets:"


class RosbagPresets:
    def __init__(self, path):
        self.path = os.path.realpath(path)  # the repo file behind the install symlink
        self._lock = threading.Lock()

    def _load(self):
        with open(self.path) as f:
            text = f.read()
        return text[:text.index(HEADER_END)] if HEADER_END in text else "", yaml.safe_load(text) or {"presets": []}

    def list(self):
        return self._load()[1]["presets"]

    def get(self, preset_id):
        p = next((p for p in self.list() if p["id"] == preset_id), None)
        if p is None:
            raise KeyError(preset_id)
        return p

    def save(self, preset, original_id=None):
        """Create (original_id None) or update a preset; returns it. Validates and normalizes the fields."""
        title = str(preset.get("title", "")).strip()
        if not title:
            raise ValueError("title is required")
        prefix = re.sub(r"[^\w-]+", "_", str(preset.get("prefix") or title).strip()).strip("_").lower()
        topics = preset.get("topics", [])
        if isinstance(topics, str):
            topics = topics.split()
        topics = [t.strip() for t in topics if t.strip()]
        all_topics = bool(preset.get("all_topics"))
        if not all_topics and not topics:
            raise ValueError("add at least one topic, or record all topics")
        with self._lock:
            header, data = self._load()
            presets = data["presets"]
            pid = original_id or re.sub(r"[^\w]+", "_", title.lower()).strip("_")
            if original_id is None:
                base, n = pid, 2
                while any(p["id"] == pid for p in presets):
                    pid, n = f"{base}_{n}", n + 1
            new = {"id": pid, "title": title, "prefix": prefix}
            old = next((p for p in presets if p["id"] == pid), None)
            if old and old.get("source"):
                new["source"] = old["source"]
            new.update({"all_topics": all_topics, "topics": [] if all_topics else topics})
            if old:
                presets[presets.index(old)] = new
            elif original_id is not None:
                raise KeyError(original_id)
            else:
                presets.append(new)
            with open(self.path, "w") as f:
                f.write(header + yaml.safe_dump(data, sort_keys=False, width=120))
        return new

    def delete(self, preset_id):
        with self._lock:
            header, data = self._load()
            before = len(data["presets"])
            data["presets"] = [p for p in data["presets"] if p["id"] != preset_id]
            if len(data["presets"]) == before:
                raise KeyError(preset_id)
            with open(self.path, "w") as f:
                f.write(header + yaml.safe_dump(data, sort_keys=False, width=120))


def record_command(preset, bags_dir):
    """(bag folder, command) for recording a preset into bags_dir on the target."""
    bag_dir = os.path.join(bags_dir, f"{preset['prefix']}_{time.strftime('%m_%d__%H_%M_%S')}")
    topics = ["--all"] if preset.get("all_topics") else [shlex.quote(t) for t in preset["topics"]]
    return bag_dir, f"ros2 bag record -o {shlex.quote(bag_dir)} " + " ".join(topics)


def storage(target, bag_dirs, bags_dir):
    """{bag_dir: bytes} for existing bag folders + free bytes on the bags drive, in one command."""
    dirs = " ".join(shlex.quote(d) for d in bag_dirs)
    cmd = (f"for d in {dirs}; do [ -d \"$d\" ] && du -sb \"$d\"; done; " if bag_dirs else "") + \
        f"echo FREE $(df -B1 --output=avail {shlex.quote(bags_dir)} | tail -1)"
    rc, out, _ = target.run(cmd, timeout=15)
    sizes, free = {}, None
    for line in out.splitlines():
        if line.startswith("FREE "):
            free = int(line.split()[1]) if line.split()[1:] and line.split()[1].isdigit() else None
        elif "\t" in line:
            size, path = line.split("\t", 1)
            sizes[path] = int(size)
    return sizes, free
