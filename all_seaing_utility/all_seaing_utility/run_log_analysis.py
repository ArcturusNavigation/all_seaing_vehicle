#!/usr/bin/env python3
"""
Analysis tool for the per-run logs written by all_seaing_common's RunLogger (issue #501).

Loads every <node>.jsonl of a run, merges them into one timeline, and either prints a
filtered timeline, per-node statistics, or the output of node/task-specific analyzers.

Analyzers are plugins: every module in all_seaing_utility/run_log_analyzers/ that defines
    NAME = "<name>"                      # used with -a/--analyzer
    DESCRIPTION = "<one line>"
    def analyze(entries, args) -> str    # entries: merged, time-sorted list of dicts
is picked up automatically. To answer a new question after a test, add a module there;
the core doesn't change.

Usage:
    ros2 run all_seaing_utility run_log_analysis.py [RUN]          # RUN: run folder or id, default latest
        [--stats]                                                  # per-node counts, rates, levels, gaps, drops
        [--timeline] [--node N ...] [--level WARN] [--event E ...] [--grep TEXT]
        [--since S] [--until S]                                    # seconds from run start
        [-a ANALYZER ...] [--list-analyzers]
With no output option, --stats is printed. Log root: $ARCTURUS_LOG_DIR (default ~/arcturus_logs).
"""

from argparse import ArgumentParser
import importlib
import json
import os
import pkgutil
import sys

from all_seaing_common.run_logger import log_dir
import all_seaing_utility.run_log_analyzers as analyzers_pkg

LEVELS = ("DEBUG", "INFO", "WARN", "ERROR")


def resolve_run(run):
    """Run folder from a path, a run id under the log root, or the latest run if None."""
    if run and os.path.isdir(run):
        return run
    root = log_dir()
    if run:
        path = os.path.join(root, run)
        if os.path.isdir(path):
            return path
        sys.exit(f"Run '{run}' not found (looked in {root})")
    runs = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))) if os.path.isdir(root) else []
    if not runs:
        sys.exit(f"No runs found in {root}")
    return os.path.join(root, runs[-1])


def _assign_times(file_entries):
    """
    Set e["_time"] (ROS time [s]) for the entries of one node file. Entries logged without a
    ROS time (e.g. the logger's own drop reports) get their wall time shifted by the node's
    ROS-minus-wall offset, taken from the nearest earlier (else later) entry. This keeps
    sim time (rosbag replay, simulation) and wall time from being mixed.
    """
    offset = None
    pending = []
    for e in file_entries:
        if e.get("t"):  # null or 0 -> missing
            offset = e["t"] - e.get("wall", e["t"])
            e["_time"] = e["t"]
            for p in pending:  # entries before the first timed one
                p["_time"] = p.get("wall", 0.0) + offset
            pending = []
        elif offset is not None:
            e["_time"] = e.get("wall", 0.0) + offset
        else:
            pending.append(e)
    for p in pending:  # no timed entry in the whole file
        p["_time"] = p.get("wall", 0.0)
    return file_entries


def load_run(run_dir):
    """All entries of a run, sorted by time (ROS time, falling back to wall time), plus a parse-error count."""
    entries, bad = [], 0
    for name in sorted(os.listdir(run_dir)):
        if not name.endswith(".jsonl"):
            continue
        file_entries = []
        with open(os.path.join(run_dir, name), errors="replace") as f:
            for line in f:
                try:
                    file_entries.append(json.loads(line))
                except json.JSONDecodeError:
                    bad += 1  # e.g. a truncated last line after a power loss
        entries.extend(_assign_times(file_entries))
    entries.sort(key=lambda e: e["_time"])
    if entries:
        t0 = entries[0]["_time"]
        for e in entries:
            e["_rel"] = e["_time"] - t0  # seconds since the first entry of the run
    return entries, bad


def filter_entries(entries, args):
    min_level = LEVELS.index(args.level) if args.level else 0
    out = []
    for e in entries:
        if args.node and e.get("node") not in args.node:
            continue
        if args.event and e.get("event") not in args.event:
            continue
        if LEVELS.index(e.get("level", "INFO")) < min_level:
            continue
        if args.since is not None and e["_rel"] < args.since:
            continue
        if args.until is not None and e["_rel"] > args.until:
            continue
        if args.grep and args.grep not in json.dumps(e):
            continue
        out.append(e)
    return out


def format_entry(e):
    fields = {k: v for k, v in e.items() if k not in ("t", "wall", "node", "level", "event") and not k.startswith("_")}
    return f"{e['_rel']:9.3f}  {e.get('level', ''):5}  {e.get('node', ''):28}  {e.get('event', ''):28}  " + \
        json.dumps(fields, separators=(",", ":"))


def stats(entries, gap_threshold=2.0):
    """Per node: entry count, mean rate, level counts, logger drops, and silent gaps."""
    by_node = {}
    for e in entries:
        by_node.setdefault(e.get("node", "?"), []).append(e)
    lines = [f"{'node':30} {'entries':>8} {'rate[Hz]':>9} {'WARN':>6} {'ERROR':>6} {'dropped':>8}  gaps>{gap_threshold:g}s"]
    for node, es in sorted(by_node.items()):
        span = es[-1]["_time"] - es[0]["_time"]
        levels = {lvl: sum(1 for e in es if e.get("level") == lvl) for lvl in LEVELS}
        dropped = sum(e.get("dropped_queue", 0) + e.get("dropped_io", 0) for e in es if e.get("event") == "run_logger_dropped")
        gaps = [(a["_rel"], b["_time"] - a["_time"]) for a, b in zip(es, es[1:]) if b["_time"] - a["_time"] > gap_threshold]
        gap_str = ", ".join(f"{t:.1f}s(+{d:.1f}s)" for t, d in gaps[:5]) + (" ..." if len(gaps) > 5 else "")
        lines.append(f"{node:30} {len(es):8d} {len(es) / span if span > 0 else 0:9.1f} "
                     f"{levels['WARN']:6d} {levels['ERROR']:6d} {dropped:8d}  {gap_str}")
    return "\n".join(lines)


def discover_analyzers():
    """{NAME: module} for every analyzer module in the run_log_analyzers package."""
    found = {}
    for info in pkgutil.iter_modules(analyzers_pkg.__path__):
        module = importlib.import_module(f"{analyzers_pkg.__name__}.{info.name}")
        if hasattr(module, "NAME") and hasattr(module, "analyze"):
            found[module.NAME] = module
    return found


def main():
    parser = ArgumentParser(description="analyze per-run node logs (RunLogger JSON lines)")
    parser.add_argument("run", nargs="?", default=None, help="run folder or run id (default: latest run)")
    parser.add_argument("--stats", action="store_true", help="per-node statistics")
    parser.add_argument("--timeline", action="store_true", help="print the (filtered) merged timeline")
    parser.add_argument("--node", nargs="+", help="only these nodes")
    parser.add_argument("--event", nargs="+", help="only these events")
    parser.add_argument("--level", choices=LEVELS, help="minimum level")
    parser.add_argument("--grep", help="only entries containing this text")
    parser.add_argument("--since", type=float, help="start time [s from run start]")
    parser.add_argument("--until", type=float, help="end time [s from run start]")
    parser.add_argument("--gap", type=float, default=2.0, help="silence threshold for --stats gaps [s]")
    parser.add_argument("-a", "--analyzer", nargs="+", help="analyzers to run")
    parser.add_argument("--list-analyzers", action="store_true", help="list available analyzers")
    args = parser.parse_args()

    available = discover_analyzers()
    if args.list_analyzers:
        for name, module in sorted(available.items()):
            print(f"{name:20} {getattr(module, 'DESCRIPTION', '')}")
        return

    run_dir = resolve_run(args.run)
    entries, bad = load_run(run_dir)
    print(f"Run: {run_dir}  ({len(entries)} entries" + (f", {bad} unparsable lines" if bad else "") + ")\n")
    selected = filter_entries(entries, args)

    if args.timeline:
        for e in selected:
            print(format_entry(e))
    for name in args.analyzer or []:
        if name not in available:
            sys.exit(f"Unknown analyzer '{name}'. Available: {', '.join(sorted(available))}")
        print(f"===== {name} =====")
        print(available[name].analyze(selected, args))
    if args.stats or not (args.timeline or args.analyzer):
        print(stats(selected, args.gap))


if __name__ == "__main__":
    main()
