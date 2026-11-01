"""
Processes started from the control panel (launch files, scripts, bag recordings...).

Each process runs detached on its target (see targets.py), so it survives panel restarts.
The list is persisted in ~/.arcturus_control_panel/processes.json, and output goes to a log
file on the target, read incrementally by the UI.
Stopping sends SIGINT to the process group (launch then shuts its nodes down cleanly),
then SIGTERM/SIGKILL if still alive after a grace period. Processes started together with another
one (args.linked_to = its id, e.g. a rosbag recording) are stopped with it, also when it exits.
"""

import json
import os
import threading
import time
import uuid

STATE_DIR = os.path.expanduser("~/.arcturus_control_panel")
STATE_FILE = os.path.join(STATE_DIR, "processes.json")


class ProcessManager:
    def __init__(self, targets):
        self.targets = targets
        self._lock = threading.Lock()
        self._procs = {}
        os.makedirs(STATE_DIR, exist_ok=True)
        try:
            with open(STATE_FILE) as f:
                self._procs = {p["id"]: p for p in json.load(f)}
        except (OSError, ValueError):
            pass

    def _save(self):
        with open(STATE_FILE, "w") as f:
            json.dump(list(self._procs.values()), f, indent=1)

    def start(self, target_name, title, cmd, action_id=None, args=None):
        """Start cmd detached on the target; args are kept with the process (e.g. linked_to)."""
        target = self.targets[target_name]
        pid_id = time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:4]
        log_path = os.path.join(target.path("panel") or "/tmp/arcturus_control_panel", f"{pid_id}.log")
        pid = target.start(cmd, log_path)
        proc = {"id": pid_id, "title": title, "action": action_id, "args": args or {}, "target": target_name,
                "cmd": cmd, "pid": pid, "log": log_path, "started": time.time(), "status": "running"}
        with self._lock:
            self._procs[pid_id] = proc
            self._save()
        return proc

    def refresh(self):
        """Update the status of running processes."""
        with self._lock:
            running = [p for p in self._procs.values() if p["status"] in ("running", "stopping")]
        for p in running:
            target = self.targets.get(p["target"])
            if target and not target.alive(p["pid"]):
                p["status"] = "exited" if p["status"] == "running" else "stopped"
                p["ended"] = time.time()
                self._stop_linked(p["id"])
        with self._lock:
            self._save()

    def _stop_linked(self, proc_id):
        for child in list(self._procs.values()):
            if child.get("args", {}).get("linked_to") == proc_id and child["status"] == "running":
                self.stop(child["id"])

    def list(self):
        with self._lock:
            return sorted(self._procs.values(), key=lambda p: p["started"], reverse=True)

    def stop(self, proc_id, grace=8.0):
        p = self._procs[proc_id]
        target = self.targets[p["target"]]
        p["status"] = "stopping"
        threading.Thread(target=self._stop_escalate, args=(p, target, grace), daemon=True).start()
        self._stop_linked(proc_id)
        return p

    def _stop_escalate(self, p, target, grace):
        for sig in ("INT", "TERM", "KILL"):
            target.signal(p["pid"], sig)
            deadline = time.time() + grace
            while time.time() < deadline:
                if not target.alive(p["pid"]):
                    p["status"], p["ended"] = "stopped", time.time()
                    with self._lock:
                        self._save()
                    return
                time.sleep(0.5)

    def read_log(self, proc_id, offset=0):
        p = self._procs[proc_id]
        data, new_offset = self.targets[p["target"]].read_file(p["log"], offset)
        return data.decode(errors="replace"), new_offset

    def remove(self, proc_id):
        with self._lock:
            p = self._procs.get(proc_id)
            if p and p["status"] not in ("running", "stopping"):
                del self._procs[proc_id]
                self._save()
