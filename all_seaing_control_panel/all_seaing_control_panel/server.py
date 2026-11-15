"""
HTTP API of the control panel (FastAPI) + the static web UI.

All blocking work (SSH commands, ROS queries) runs in FastAPI's thread pool (sync `def`
endpoints), so one slow command never freezes the UI.
"""

import json
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


def timed_entries(entries):
    """Run-log entries from several nodes -> sorted, with `_time` (ROS time [s], per node) and `_rel`
    (seconds since the first entry), as the analyzers expect."""
    from all_seaing_utility import run_log_analysis as rla
    by_node = {}
    for e in entries:
        by_node.setdefault(e.get("node"), []).append(e)
    out = []
    for es in by_node.values():
        out.extend(rla._assign_times(es))
    out.sort(key=lambda e: e["_time"])
    t0 = out[0]["_time"] if out else 0
    for e in out:
        e["_rel"] = e["_time"] - t0
    return out


def analyzer_view(analyzer, entries):
    """Map-player data of an analyzer that defines view() (slam, tasks)."""
    from argparse import Namespace
    from all_seaing_utility import run_log_analysis as rla
    module = rla.discover_analyzers().get(analyzer)
    if module is None or not hasattr(module, "view"):
        raise HTTPException(404, f"analyzer {analyzer} has no map view")
    return module.view(entries, Namespace())


def analyzer_report(analyzer, entries):
    """Run one analyzer on entries: text output, structured summary (if it defines summary()) and plots."""
    from argparse import Namespace
    from all_seaing_utility import run_log_analysis as rla
    module = rla.discover_analyzers().get(analyzer)
    if module is None:
        raise HTTPException(404, f"unknown analyzer {analyzer}")
    if not entries:
        return {"analyzer": analyzer, "entries": 0, "output": "no entries", "summary": None, "plots": []}
    args = Namespace()
    return {"analyzer": analyzer, "entries": len(entries), "output": module.analyze(entries, args),
            "summary": module.summary(entries, args) if hasattr(module, "summary") else None,
            "plots": module.plots(entries, args) if hasattr(module, "plots") else []}


def create_app(ctx):
    """ctx: object with targets, registry, processes, monitor, checks, params, runs, cfg, static_dir."""
    app = FastAPI(title="Arcturus Control Panel")

    def target(name):
        if name not in ctx.targets:
            raise HTTPException(404, f"unknown target {name}")
        return ctx.targets[name]

    # ------------------------------------------------------------------ general

    @app.get("/api/info")
    def info():
        return {"targets": [{"name": n, "label": t.label, "kind": t.kind, "paths": t.paths}
                            for n, t in ctx.targets.items()],
                "default_target": ctx.cfg.get("default_target", next(iter(ctx.targets))),
                "links": ctx.cfg.get("links", [])}

    @app.get("/api/targets/{name}/status")
    def target_status(name):
        t = target(name)
        rc, out, err = t.run("hostname", timeout=8)
        return {"reachable": rc == 0, "hostname": out.strip(), "error": err.strip() if rc else ""}

    # ------------------------------------------------------------------ run page

    @app.get("/api/registry")
    def registry():
        return ctx.registry.public()

    @app.get("/api/choices/{kind}")
    def choices(kind: str, target_name: str):
        """Dynamic argument choices from the target: bag / run."""
        t = target(target_name)
        path = {"bag": t.path("bags"), "run": t.path("logs")}.get(kind)
        if path is None:
            raise HTTPException(400, f"unknown choice kind {kind}")
        rc, out, _ = t.run(f"ls -1t {shlex.quote(path)} 2>/dev/null", timeout=10)
        names = [n for n in out.strip().splitlines() if n]
        return [os.path.join(path, n) if kind == "bag" else n for n in names]

    @app.post("/api/actions/{action_id}/preview")
    def preview(action_id: str, payload: dict = Body(default={})):
        t = ctx.targets.get(payload.get("target") or ctx.cfg.get("default_target"))
        return {"command": ctx.registry.build_command(action_id, payload.get("values", {}), t.paths if t else None)}

    @app.post("/api/actions/{action_id}/start")
    def start(action_id: str, payload: dict = Body(...)):
        action = ctx.registry.actions.get(action_id)
        if action is None:
            raise HTTPException(404, "unknown action")
        values = payload.get("values", {})
        tname = payload.get("target") or action.get("target") or ctx.cfg.get("default_target")
        display_cmd = ctx.registry.build_command(action_id, values, target(tname).paths)
        cmd = display_cmd
        stdin = None
        if action.get("sudo"):
            # the password goes through stdin only: never in the command line, logs or process list
            cmd = f"sudo -S -p '' bash -c {shlex.quote(display_cmd)}"
            stdin = payload.get("sudo_password", "") + "\n"
            display_cmd = "sudo " + display_cmd
        if action.get("mode") == "blocking" or stdin is not None:  # short commands: run, return the output
            rc, out, err = target(tname).run(cmd, timeout=action.get("timeout", 120), stdin=stdin)
            return {"blocking": True, "command": display_cmd, "returncode": rc, "output": out + err}
        title = action["title"] + (" (" + ", ".join(f"{k}={v}" for k, v in values.items() if v not in ("", None)) + ")"
                                   if values else "")
        proc = ctx.processes.start(tname, title, cmd, action_id, values)
        if payload.get("record_preset"):  # "record rosbag" option: a recording stopped together with it
            proc["recording"] = start_recording(payload["record_preset"], tname, linked_to=proc["id"])
        return proc

    # ------------------------------------------------------------------ rosbag recording (Run tab)

    def start_recording(preset_id, tname, linked_to=None):
        from all_seaing_control_panel.rosbags import record_command
        try:
            preset = ctx.rosbag_presets.get(preset_id)
        except KeyError:
            raise HTTPException(404, f"no rosbag preset {preset_id}")
        t = target(tname)
        if not t.path("bags"):
            raise HTTPException(400, f"no bags path configured for {tname}")
        bag_dir, cmd = record_command(preset, t.path("bags"))
        return ctx.processes.start(tname, f"Rosbag: {preset['title']} -> {os.path.basename(bag_dir)}", cmd, "rosbag",
                                   {"preset": preset_id, "bag_dir": bag_dir, "linked_to": linked_to})

    @app.get("/api/rosbags/presets")
    def rosbag_presets():
        return ctx.rosbag_presets.list()

    @app.post("/api/rosbags/presets")
    def rosbag_preset_create(preset: dict = Body(...)):
        try:
            return ctx.rosbag_presets.save(preset)
        except ValueError as e:
            raise HTTPException(400, str(e))

    @app.put("/api/rosbags/presets/{preset_id}")
    def rosbag_preset_update(preset_id: str, preset: dict = Body(...)):
        try:
            return ctx.rosbag_presets.save(preset, original_id=preset_id)
        except ValueError as e:
            raise HTTPException(400, str(e))
        except KeyError:
            raise HTTPException(404, f"no rosbag preset {preset_id}")

    @app.delete("/api/rosbags/presets/{preset_id}")
    def rosbag_preset_delete(preset_id: str):
        try:
            ctx.rosbag_presets.delete(preset_id)
        except KeyError:
            raise HTTPException(404, f"no rosbag preset {preset_id}")
        return {"ok": True}

    @app.post("/api/rosbags/record")
    def rosbag_record(payload: dict = Body(...)):
        return start_recording(payload["preset"], payload.get("target") or ctx.cfg.get("default_target"))

    @app.get("/api/rosbags/status")
    def rosbag_status(target_name: str):
        """Recordings on a target (running + the 10 most recent ended) with their size, and the free space."""
        from all_seaing_control_panel.rosbags import storage
        t = target(target_name)
        procs = [p for p in ctx.processes.list() if p.get("action") == "rosbag" and p["target"] == target_name]
        procs = [p for p in procs if p["status"] in ("running", "stopping")] + \
            [p for p in procs if p["status"] not in ("running", "stopping")][:10]
        sizes, free = storage(t, [p["args"]["bag_dir"] for p in procs], t.path("bags"))
        return {"free_bytes": free, "bags_path": t.path("bags"),
                "recordings": [{"id": p["id"], "status": p["status"], "preset": p["args"].get("preset"),
                                "bag_dir": p["args"]["bag_dir"], "linked_to": p["args"].get("linked_to"),
                                "started": p["started"], "ended": p.get("ended"),
                                "bytes": sizes.get(p["args"]["bag_dir"])} for p in procs]}

    @app.get("/api/processes")
    def processes():
        ctx.processes.refresh()
        return ctx.processes.list()

    @app.post("/api/processes/{proc_id}/stop")
    def stop(proc_id: str):
        return ctx.processes.stop(proc_id)

    @app.delete("/api/processes/{proc_id}")
    def remove(proc_id: str):
        ctx.processes.remove(proc_id)
        return {"ok": True}

    @app.get("/api/processes/{proc_id}/log")
    def log(proc_id: str, offset: int = 0):
        text, new_offset = ctx.processes.read_log(proc_id, offset)
        return {"text": text, "offset": new_offset}

    # ------------------------------------------------------------------ health & checks

    @app.get("/api/diagnostics")
    def diagnostics():
        return ctx.monitor.diagnostics_snapshot()

    # ------------------------------------------------------------------ live run log (via foxglove_bridge)

    def live(target_name):
        """LiveLog stream of a target (started on first use), from its foxglove_url."""
        t = target(target_name)
        url = ctx.cfg["targets"][target_name].get("foxglove_url")
        if not url:
            raise HTTPException(400, f"no foxglove_url configured for {target_name}")
        if target_name not in ctx.live_logs:
            from all_seaing_control_panel.live_log import LiveLog
            ctx.live_logs[target_name] = LiveLog(url)
        return ctx.live_logs[target_name]

    def live_entries(target_name, window):
        return timed_entries(live(target_name).window(window))

    @app.get("/api/live/status")
    def live_status(target_name: str):
        return live(target_name).status()

    @app.get("/api/live/entries")
    def live_feed(target_name: str, window: float = 30, level: str = "", node: str = "", event: str = "",
                  grep: str = "", limit: int = 200):
        levels = ("DEBUG", "INFO", "WARN", "ERROR")
        min_level = levels.index(level) if level in levels else 0
        out = [e for e in live_entries(target_name, window)
               if levels.index(e.get("level", "INFO")) >= min_level and (not node or node in e.get("node", ""))
               and (not event or event in e.get("event", "")) and (not grep or grep in json.dumps(e))]
        return [{k: v for k, v in e.items() if k != "_time"} for e in out[-limit:]]

    @app.get("/api/live/analyze")
    def live_analyze(target_name: str, analyzer: str, window: float = 30):
        return analyzer_report(analyzer, live_entries(target_name, window))

    @app.get("/api/live/view")
    def live_view_map(target_name: str, analyzer: str, window: float = 30):
        return analyzer_view(analyzer, live_entries(target_name, window))

    # ------------------------------------------------------------------ recordings (laptop-side MCAP)

    def bridge_url(target_name):
        url = ctx.cfg["targets"][target(target_name).name].get("foxglove_url")
        if not url:
            raise HTTPException(400, f"no foxglove_url configured for {target_name}")
        return url

    def recorder(target_name):
        if target_name not in ctx.recorders:
            from all_seaing_control_panel.recorder import Recorder
            ctx.recorders[target_name] = Recorder(bridge_url(target_name), ctx.recordings_cfg)
        return ctx.recorders[target_name]

    def live_view(target_name):
        if target_name not in ctx.live_views:
            from all_seaing_control_panel.live_view import LiveView
            ctx.live_views[target_name] = LiveView(bridge_url(target_name), ctx.recordings_cfg)
        return ctx.live_views[target_name]

    @app.get("/api/liveview/frame")
    def live_view_frame(target_name: str, camera: str):
        """Latest frame of a camera through the bridge (nothing recorded); subscribes while polled."""
        return live_view(target_name).latest(camera)

    def recordings():
        if not hasattr(ctx, "recordings"):
            from all_seaing_control_panel.recordings import Recordings
            ctx.recordings = Recordings(ctx.recordings_cfg)
        return ctx.recordings

    @app.get("/api/recorder")
    def recorder_status(target_name: str):
        return recorder(target_name).status()

    @app.post("/api/recorder/{action}")
    def recorder_control(action: str, target_name: str):
        r = recorder(target_name)
        if action == "start":
            r.start()
        elif action == "stop":
            r.stop()
        else:
            raise HTTPException(400, "action must be start or stop")
        return r.status()

    @app.get("/api/recordings")
    def recordings_list():
        active = [p for p in (r.current_segment() for r in ctx.recorders.values()) if p]
        return {"sources": recordings().sources(active), "cameras": ctx.recordings_cfg["cameras"]}

    @app.get("/api/bags")
    def bags(target_name: str):
        """Rosbags (folders with metadata.yaml) in the target's bags path, newest first, with their size."""
        t = target(target_name)
        path = t.path("bags")
        rc, out, _ = t.run(f"cd {shlex.quote(path)} 2>/dev/null && for d in $(ls -1t); do "
                           f"[ -f \"$d/metadata.yaml\" ] && echo \"$(du -sb \"$d\" | cut -f1) $(stat -c %Y \"$d\") $d\"; done",
                           timeout=60)
        out_list = []
        for line in out.splitlines():
            size, mtime, name = line.split(" ", 2)
            out_list.append({"path": os.path.join(path, name), "name": name, "bytes": int(size), "mtime": int(mtime)})
        return out_list

    @app.post("/api/recordings/import")
    def recording_import(payload: dict = Body(...)):
        """View a rosbag of a target: convert (on the target) -> download -> MCAP recording. Returns the job."""
        tname = payload.get("target") or ctx.cfg.get("default_target")
        target(tname)
        return ctx.bag_imports.start(tname, payload["bag"], payload.get("image_rate", 5.0), payload.get("cloud_rate", 5.0))

    @app.get("/api/recordings/imports")
    def recording_imports():
        return ctx.bag_imports.list()

    @app.get("/api/recordings/{source}/timeline")
    def recording_timeline(source: str):
        return recordings().timeline(source)

    @app.post("/api/recordings/save")
    def recording_save(payload: dict = Body(...)):
        try:
            from all_seaing_control_panel.recordings import BUFFER
            return recordings().save(float(payload["t0"]), float(payload["t1"]), payload.get("name", "recording"),
                                     payload.get("source", BUFFER))
        except FileExistsError as e:
            raise HTTPException(409, str(e))

    @app.delete("/api/recordings/{source}")
    def recording_delete(source: str):
        """Delete a saved recording or a buffer session (keeping the segments recorders are writing)."""
        keep = [p for p in (r.current_segment() for r in ctx.recorders.values()) if p]
        try:
            return {"removed": recordings().delete(source, keep)}
        except FileNotFoundError:
            raise HTTPException(404, f"no recording {source}")

    @app.get("/api/recordings/{source}/frames")
    def recording_frames(source: str, camera: str):
        return recordings().frame_list(source, camera)

    @app.get("/api/recordings/{source}/frame")
    def recording_frame(source: str, camera: str, index: int):
        return recordings().frame(source, camera, index)

    @app.get("/api/recordings/{source}/report")
    def recording_report(source: str, analyzer: str, t0: float = None, t1: float = None):
        return analyzer_report(analyzer, timed_entries(recordings().run_log_entries(source, t0, t1)))

    @app.get("/api/recordings/{source}/view")
    def recording_view(source: str, analyzer: str, t0: float = None, t1: float = None):
        return analyzer_view(analyzer, timed_entries(recordings().run_log_entries(source, t0, t1)))

    @app.get("/api/descriptions")
    def descriptions():
        """How to present diagnostics (config/descriptions.yaml)."""
        return ctx.descriptions

    @app.get("/api/checks")
    def checks_list():
        return ctx.checks.list()

    @app.post("/api/checks/{check_id}")
    def check_run(check_id: str, target_name: str, mode: str = "auto"):
        return ctx.checks.run(check_id, target_name, mode)

    @app.get("/api/mode")
    def mode(target_name: str):
        """Detected run mode on the target: live | replay | sim (from the ROS graph)."""
        detected, nodes = ctx.checks.detect_mode(target_name)
        return {"mode": detected, "node_count": len(nodes)}

    # ------------------------------------------------------------------ runs & analysis

    @app.get("/api/runs")
    def runs(target_name: str):
        return ctx.runs.list(target_name)

    @app.get("/api/runs/{run}/analyze")
    def analyze(run: str, target_name: str, analyzers: str = "", extra: str = ""):
        return {"output": ctx.runs.analyze(target_name, run, [a for a in analyzers.split(",") if a], extra)}

    @app.get("/api/runs/{run}/view")
    def run_view(run: str, target_name: str, analyzer: str):
        return analyzer_view(analyzer, ctx.runs.entries(target_name, run))

    @app.get("/api/labels")
    def labels():
        """Detection label number -> name (all_seaing_bringup all_label_mappings.yaml), for coloring maps."""
        import yaml
        path = os.path.join(get_package_share_directory("all_seaing_bringup"), "config", "perception",
                            "all_label_mappings.yaml")
        with open(path) as f:
            return {int(v): k for k, v in (yaml.safe_load(f) or {}).items()}

    @app.get("/api/runs/{run}/report")
    def run_report(run: str, target_name: str, analyzer: str):
        """Analyzer on a past run (computed on the laptop from the fetched logs, same code as the CLI)."""
        return analyzer_report(analyzer, ctx.runs.entries(target_name, run))

    @app.get("/api/runs/{run}/catalog")
    def catalog(run: str, target_name: str):
        return ctx.runs.catalog(target_name, run)

    @app.post("/api/runs/{run}/plots")
    def plots(run: str, target_name: str, spec: dict = Body(...)):
        try:
            return ctx.runs.plots(target_name, run, spec)
        except (KeyError, ValueError) as e:
            raise HTTPException(400, str(e))

    @app.get("/api/analyzers")
    def analyzers():
        from all_seaing_utility import run_log_analysis as rla
        return [{"name": n, "description": getattr(m, "DESCRIPTION", ""), "plots": hasattr(m, "plots")}
                for n, m in sorted(rla.discover_analyzers().items())]

    # ------------------------------------------------------------------ parameters

    @app.get("/api/params")
    def params(target_name: str):
        return ctx.params.values(target_name)

    @app.post("/api/params/{param_id}")
    def param_set(param_id: str, target_name: str, payload: dict = Body(...)):
        try:
            return ctx.params.set(target_name, param_id, payload["value"], payload.get("live", True))
        except (ValueError, SyntaxError) as e:
            raise HTTPException(400, f"invalid value: {e}")

    @app.delete("/api/params/{param_id}")
    def param_clear(param_id: str, target_name: str):
        ctx.params.clear(target_name, param_id)
        return {"ok": True}

    @app.get("/api/params-apply/preview")
    def apply_preview(target_name: str):
        plan = ctx.params.plan_apply(target_name)
        return {"diffs": plan["diffs"], "problems": plan["problems"]}

    @app.post("/api/params-apply")
    def apply(target_name: str):
        return ctx.params.apply(target_name)

    # ------------------------------------------------------------------ UI

    # Offline satellite tiles (all_seaing_bringup/tile/{z}/{x}/{y}.png) for Foxglove's Map panel
    # (custom tile URL http://localhost:8080/tiles/{z}/{x}/{y}.png), replacing RViz's AerialMap
    tile_dir = os.path.join(get_package_share_directory("all_seaing_bringup"), "tile")
    if os.path.isdir(tile_dir):
        app.mount("/tiles", StaticFiles(directory=tile_dir, follow_symlink=True), name="tiles")

    # follow_symlink: with `colcon build --symlink-install` the installed files are symlinks to the source tree
    app.mount("/static", StaticFiles(directory=ctx.static_dir, follow_symlink=True), name="static")

    @app.get("/")
    def index():
        return FileResponse(os.path.join(ctx.static_dir, "index.html"))

    return app
