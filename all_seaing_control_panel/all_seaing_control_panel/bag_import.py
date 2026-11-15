"""
Viewing existing rosbags in the Recordings tab, without `ros2 bag play`: a bag on a target (e.g.
the Jetson's NVMe) is turned into a saved recording (MCAP on the laptop) in three phases, each
reported as progress:

1. convert (on the target, next to the bag, so the raw data never crosses Wi-Fi):
   `rosbag_light_copy.py` (all_seaing_utility) writes a light copy: JPEG images at <= image_rate,
   point clouds at <= cloud_rate (whole LiDAR revolutions), everything else unchanged, plus the
   message definitions (schemas.json). The raw /velodyne_points is left out when the bag also has
   /point_cloud/filtered (the cloud fusion uses, and the one the viewer shows first). It runs as a panel process (visible/stoppable in the Run
   tab) into <bags path>/../panel_previews/<bag>_light/, and is reused if already there.
2. download the light copy (SFTP).
3. write it as <saved_dir>/<bag>.mcap (ZSTD), read with the sqlite3 module (no ROS needed), then
   delete the downloaded copy and the target's light copy.

Jobs are kept in memory: {id, bag, target, phase, progress (0..1 overall), status
running|done|error, error, result (saved recording id)}.
"""

import json
import os
import re
import shlex
import shutil
import sqlite3
import threading
import time
import uuid

from mcap.writer import CompressionType, Writer

PHASES = {"convert": (0.0, 0.6), "download": (0.6, 0.9), "mcap": (0.9, 1.0)}  # share of the overall bar


class BagImports:
    def __init__(self, processes, targets, saved_dir, work_dir):
        self.processes = processes
        self.targets = targets
        self.saved_dir = os.path.expanduser(saved_dir)
        self.work_dir = os.path.expanduser(work_dir)
        self.jobs = {}

    def list(self):
        return sorted(self.jobs.values(), key=lambda j: j["started"], reverse=True)

    def start(self, target_name, bag, image_rate=5.0, cloud_rate=5.0):
        name = os.path.basename(bag.rstrip("/"))
        for j in self.jobs.values():
            if j["bag"] == bag and j["status"] == "running":
                return j
        job = {"id": uuid.uuid4().hex[:8], "bag": bag, "name": name, "target": target_name, "phase": "convert",
               "progress": 0.0, "status": "running", "error": "", "result": None, "started": time.time(),
               "image_rate": float(image_rate), "cloud_rate": float(cloud_rate)}
        self.jobs[job["id"]] = job
        threading.Thread(target=self._run, args=(job,), daemon=True, name=f"import {name}").start()
        return job

    def _set(self, job, phase, fraction):
        lo, hi = PHASES[phase]
        job["phase"], job["progress"] = phase, round(lo + (hi - lo) * min(1.0, max(0.0, fraction)), 4)

    def _run(self, job):
        try:
            t = self.targets[job["target"]]
            remote = os.path.join(os.path.dirname(t.path("bags").rstrip("/")), "panel_previews", f"{job['name']}_light")
            self._convert(job, t, remote)
            local = os.path.join(self.work_dir, job["name"])
            self._download(job, t, remote, local)
            job["result"] = self._to_mcap(job, local)
            shutil.rmtree(local, ignore_errors=True)
            t.run(f"rm -rf {shlex.quote(remote)}", timeout=60)
            job["status"], job["progress"] = "done", 1.0
        except Exception as e:
            job["status"], job["error"] = "error", str(e)

    # ------------------------------------------------------------------ 1. convert on the target

    def _convert(self, job, t, remote):
        q = shlex.quote
        rc, out, _ = t.run(f"test -f {q(remote)}/schemas.json && test -f {q(remote)}/metadata.yaml && echo READY",
                           timeout=15)
        if "READY" in out:  # converted before (and not cleaned up): reuse
            self._set(job, "convert", 1.0)
            return
        t.run(f"rm -rf {q(remote)}; mkdir -p {q(os.path.dirname(remote))}", timeout=30)
        meta, _ = t.read_file(os.path.join(job["bag"], "metadata.yaml"), 0, max_bytes=5_000_000)
        exclude = " --exclude /velodyne_points" if b"name: /point_cloud/filtered\n" in meta else ""
        cmd = (f"ros2 run all_seaing_utility rosbag_light_copy.py -i {q(job['bag'])} -o {q(remote)} "
               f"--image-rate {job['image_rate']} --cloud-rate {job['cloud_rate']}{exclude}")
        proc = self.processes.start(job["target"], f"Light copy of {job['name']} (Recordings)", cmd, "bag_import",
                                    {"bag": job["bag"]})
        job["process"] = proc["id"]
        text = ""
        while True:
            time.sleep(1.0)
            data, _ = t.read_file(proc["log"], 0, max_bytes=50_000_000)
            text = data.decode(errors="replace")
            prog = re.findall(r"^PROGRESS ([\d.]+)", text, re.M)
            if prog:
                self._set(job, "convert", float(prog[-1]))
            if re.search(r"^DONE ", text, re.M):
                return
            self.processes.refresh()
            if next(p for p in self.processes.list() if p["id"] == proc["id"])["status"] != "running":
                lines = [l for l in text.splitlines() if not l.startswith("PROGRESS")]
                raise RuntimeError("conversion failed:\n" + "\n".join(lines[-15:]))

    # ------------------------------------------------------------------ 2. download

    def _download(self, job, t, remote, local):
        rc, out, err = t.run(f"cd {shlex.quote(remote)} && for f in *; do echo \"$(stat -c %s \"$f\") $f\"; done",
                             timeout=30)
        files = [(int(l.split(" ", 1)[0]), l.split(" ", 1)[1]) for l in out.splitlines() if " " in l]
        if rc != 0 or not files:
            raise RuntimeError(f"light copy not found on {job['target']}: {err.strip()}")
        shutil.rmtree(local, ignore_errors=True)
        os.makedirs(local)
        total, done = sum(s for s, _ in files) or 1, 0
        for size, name in files:
            t.download(os.path.join(remote, name), os.path.join(local, name),
                       lambda d, _total, base=done: self._set(job, "download", (base + d) / total))
            done += size

    # ------------------------------------------------------------------ 3. MCAP

    def _to_mcap(self, job, local):
        with open(os.path.join(local, "schemas.json")) as f:
            schemas = json.load(f)
        dbs = sorted(p for p in os.listdir(local) if p.endswith(".db3"))
        os.makedirs(self.saved_dir, exist_ok=True)
        out_name = job["name"] + ".mcap"
        if os.path.exists(os.path.join(self.saved_dir, out_name)):
            out_name = f"{job['name']}_{time.strftime('%H%M%S')}.mcap"
        path = os.path.join(self.saved_dir, out_name)
        total = sum(sqlite3.connect(os.path.join(local, db)).execute("select count(*) from messages").fetchone()[0]
                    for db in dbs) or 1
        n = 0
        with open(path + ".part", "wb") as f:
            w = Writer(f, compression=CompressionType.ZSTD)
            w.start(profile="ros2", library="arcturus_control_panel")
            schema_ids, channels = {}, {}
            for db in dbs:
                con = sqlite3.connect(os.path.join(local, db))
                topics = {tid: (name, ttype) for tid, name, ttype in con.execute("select id, name, type from topics")}
                for tid, (name, ttype) in topics.items():
                    if name not in channels:
                        if ttype not in schema_ids:
                            schema_ids[ttype] = w.register_schema(name=ttype, encoding="ros2msg",
                                                                  data=schemas.get(ttype, "").encode())
                        channels[name] = w.register_channel(topic=name, message_encoding="cdr",
                                                            schema_id=schema_ids[ttype])
                for tid, ts, data in con.execute("select topic_id, timestamp, data from messages order by timestamp"):
                    w.add_message(channel_id=channels[topics[tid][0]], log_time=ts, publish_time=ts, data=data)
                    n += 1
                    if n % 2000 == 0:
                        self._set(job, "mcap", n / total)
                con.close()
            w.finish()
        os.replace(path + ".part", path)
        return out_name
