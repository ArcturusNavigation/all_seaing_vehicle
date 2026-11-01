#!/usr/bin/env python3
"""
Arcturus Control Panel (issue #327): a web app on the laptop to run, monitor and check the
boat (or the simulation), all in one place:
- Run:        launch files, scripts, analysis tools and recordings from a registry
              (config/registry.yaml), with argument forms, live output, stop buttons
- Health:     live /diagnostics of every component (incl. topic rates from topic_monitor)
- Checks:     pre-run checks (ports, MAVROS & rate, central hub, sensors, time sync, TF,
              heading vs GPS, disk, code version) with one-click fixes
- Runs:       per-run logs: analyzers and plots (auto dashboard)
- Parameters: view/set basic parameters, save them, write them into the launch files

Usage:
    ros2 run all_seaing_control_panel control_panel.py [--port 8080] [--config-dir DIR] [--no-browser]
then open http://localhost:8080. It only listens on localhost: it can run commands on the
boat, so it must not be reachable from the network.
"""

import argparse
import os
import webbrowser

import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory

from all_seaing_control_panel.checks import Checks
from all_seaing_control_panel.params import Params
from all_seaing_control_panel.processes import ProcessManager
from all_seaing_control_panel.registry import Registry
from all_seaing_control_panel.ros_monitor import RosMonitorThread
from all_seaing_control_panel.runs import Runs
from all_seaing_control_panel.server import create_app
from all_seaing_control_panel.targets import make_targets


class Context:
    pass


def main():
    share = get_package_share_directory("all_seaing_control_panel")
    parser = argparse.ArgumentParser(description="Arcturus control panel (web UI on localhost)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--config-dir", default=os.path.join(share, "config"))
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    def load(name):
        with open(os.path.join(args.config_dir, name)) as f:
            return yaml.safe_load(f)

    ctx = Context()
    ctx.cfg = load("targets.yaml")
    # Personal overrides (e.g. where the workspace is on this laptop), merged per target
    local_cfg_path = os.path.expanduser("~/.arcturus_control_panel/targets.yaml")
    if os.path.exists(local_cfg_path):
        with open(local_cfg_path) as f:
            local_cfg = yaml.safe_load(f) or {}
        for name, tcfg in local_cfg.get("targets", {}).items():
            base = ctx.cfg["targets"].setdefault(name, {})
            base.update({k: v for k, v in tcfg.items() if k != "paths"})
            base.setdefault("paths", {}).update(tcfg.get("paths", {}))
        ctx.cfg.update({k: v for k, v in local_cfg.items() if k != "targets"})
    ctx.targets = make_targets(ctx.cfg["targets"])
    ctx.registry = Registry(os.path.join(args.config_dir, "registry.yaml"))
    ctx.processes = ProcessManager(ctx.targets)
    ctx.monitor = RosMonitorThread(ctx.cfg.get("ros_topics", {})).node
    ctx.checks = Checks(load("checks.yaml"), ctx.targets, ctx.monitor)
    ctx.params = Params(os.path.join(args.config_dir, "parameters.yaml"), ctx.targets)
    ctx.runs = Runs(ctx.targets)
    ctx.descriptions = load("descriptions.yaml")
    ctx.live_logs = {}
    ctx.recordings_cfg = load("recordings.yaml")
    ctx.recorders = {}  # target -> Recorder (laptop-side recording through the target's foxglove_bridge)
    ctx.live_views = {}  # target -> LiveView (live frame view through the bridge, not recorded)
    from all_seaing_control_panel.bag_import import BagImports
    ctx.bag_imports = BagImports(ctx.processes, ctx.targets, ctx.recordings_cfg["saved_dir"],
                                 os.path.join(os.path.dirname(os.path.expanduser(ctx.recordings_cfg["saved_dir"])), "imports"))
    from all_seaing_control_panel.rosbags import RosbagPresets
    ctx.rosbag_presets = RosbagPresets(os.path.join(args.config_dir, "rosbag_presets.yaml"))
    ctx.static_dir = os.path.join(share, "static")

    url = f"http://localhost:{args.port}"
    print(f"Arcturus control panel: {url}")
    if not args.no_browser:
        webbrowser.open(url)
    uvicorn.run(create_app(ctx), host="127.0.0.1", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
