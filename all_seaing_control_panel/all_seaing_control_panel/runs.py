"""
Runs & analysis page (issues #327 auto dashboard, #501): browse the per-run logs on a target,
run the analysis tool (run_log_analysis.py) there, and build plots.

Plots are built on the laptop from the run's JSON-lines logs (fetched gzipped over SSH),
reusing run_log_analysis.load_run(). Plot sources:
- analyzers that define `plots(entries, args) -> [plot spec]` (e.g. fusion)
- generic builders that work for any logged data:
    event_rates:  rate of every (node, event) over time
    field:        a numeric field of an event over time (e.g. controller error, SLAM covariance)
    xy:           two numeric fields as a trajectory/map (e.g. pose x/y from different nodes)
    timeline:     a string field over time as state segments (e.g. task state)
Plot spec: {"title", "kind": "line"|"xy"|"timeline", "x_label", "y_label",
            "series": [{"name", "x": [...], "y": [...]}]  (timeline: "segments": [{"name","start","end","label"}])}
"""

import gzip
import io
import json
import os
import shlex
import tempfile
from argparse import Namespace
from collections import defaultdict

from all_seaing_utility import run_log_analysis as rla


class Runs:
    def __init__(self, targets):
        self.targets = targets
        self._cache = {}  # (target, run, size) -> entries

    def list(self, target_name):
        t = self.targets[target_name]
        logs = t.path("logs")
        rc, out, err = t.run(f"cd {shlex.quote(logs)} 2>/dev/null && for d in $(ls -1t); do "
                             f"[ -d \"$d\" ] && echo \"$d $(du -sk \"$d\" | cut -f1) $(ls \"$d\" | wc -l)\"; done",
                             timeout=20)
        runs = []
        for line in out.strip().splitlines():
            name, kb, nfiles = line.rsplit(" ", 2)
            runs.append({"name": name, "size_kb": int(kb), "files": int(nfiles)})
        return runs

    def analyze(self, target_name, run, analyzers=(), extra=""):
        """Run run_log_analysis.py on the target (same tool as on the command line)."""
        t = self.targets[target_name]
        cmd = f"ros2 run all_seaing_utility run_log_analysis.py {shlex.quote(run)}"
        if analyzers:
            cmd += " -a " + " ".join(shlex.quote(a) for a in analyzers)
        if extra:
            cmd += " " + extra
        rc, out, err = t.run(cmd, timeout=120)
        return out + (("\n" + err) if rc != 0 else "")

    def entries(self, target_name, run):
        t = self.targets[target_name]
        path = f"{t.path('logs')}/{run}"
        rc, size, _ = t.run(f"du -sb {shlex.quote(path)} | cut -f1", timeout=10)
        key = (target_name, run, size.strip())
        if key in self._cache:
            return self._cache[key]
        rc, data, err = t._raw(f"cd {shlex.quote(path)} && tar czf - *.jsonl", timeout=120, binary=True)
        if rc not in (0, 1) or not data:  # 1: "file changed as we read it" (a run still being written): fine
            raise RuntimeError(f"cannot read run {run}: {err}")
        with tempfile.TemporaryDirectory() as tmp:
            import tarfile
            with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
                tar.extractall(tmp)
            entries, _ = rla.load_run(tmp)
        self._cache = {key: entries}  # keep only the latest run in memory
        return entries

    def catalog(self, target_name, run):
        """Nodes, events and their numeric/string fields, to fill the plot builder menus."""
        cat = defaultdict(lambda: defaultdict(lambda: {"numeric": set(), "string": set()}))
        for e in self.entries(target_name, run):
            fields = cat[e.get("node", "?")][e.get("event", "?")]
            for k, v in e.items():
                if k in ("t", "wall", "node", "level", "event") or k.startswith("_"):
                    continue
                if isinstance(v, (int, float)) and not isinstance(v, bool):
                    fields["numeric"].add(k)
                elif isinstance(v, str):
                    fields["string"].add(k)
        return {n: {ev: {k: sorted(s) for k, s in f.items()} for ev, f in evs.items()} for n, evs in cat.items()}

    def plots(self, target_name, run, spec):
        entries = self.entries(target_name, run)
        kind = spec.get("kind")
        if kind == "analyzer":
            module = rla.discover_analyzers().get(spec["analyzer"])
            if module is None or not hasattr(module, "plots"):
                return []
            return module.plots(entries, Namespace())
        if kind == "event_rates":
            return [event_rates(entries, spec.get("bucket", 1.0))]
        if kind == "field":
            return [field_plot(entries, spec["node"], spec["event"], spec["fields"])]
        if kind == "xy":
            return [xy_plot(entries, spec["series"])]
        if kind == "timeline":
            return [timeline_plot(entries, spec["node"], spec["event"], spec["field"])]
        raise ValueError(f"unknown plot kind {kind}")


def event_rates(entries, bucket=1.0):
    counts = defaultdict(lambda: defaultdict(int))
    for e in entries:
        counts[f"{e.get('node')}/{e.get('event')}"][int(e["_rel"] // bucket)] += 1
    end = int(max((e["_rel"] for e in entries), default=0) // bucket) + 1
    series = [{"name": name, "x": [b * bucket for b in range(end)], "y": [c.get(b, 0) / bucket for b in range(end)]}
              for name, c in sorted(counts.items())]
    return {"title": "Event rates", "kind": "line", "x_label": "time [s]", "y_label": "events/s", "series": series}


def field_plot(entries, node, event, fields):
    series = []
    for f in fields:
        pts = [(e["_rel"], e[f]) for e in entries if e.get("node") == node and e.get("event") == event
               and isinstance(e.get(f), (int, float))]
        series.append({"name": f, "x": [p[0] for p in pts], "y": [p[1] for p in pts]})
    return {"title": f"{node} / {event}", "kind": "line", "x_label": "time [s]", "y_label": ", ".join(fields),
            "series": series}


def xy_plot(entries, series_specs):
    """series_specs: [{"node", "event", "x", "y", "name"}]."""
    series = []
    for s in series_specs:
        pts = [(e[s["x"]], e[s["y"]]) for e in entries if e.get("node") == s["node"] and e.get("event") == s["event"]
               and isinstance(e.get(s["x"]), (int, float)) and isinstance(e.get(s["y"]), (int, float))]
        series.append({"name": s.get("name") or f"{s['node']}/{s['event']}", "x": [p[0] for p in pts],
                       "y": [p[1] for p in pts]})
    return {"title": "XY", "kind": "xy", "x_label": "x", "y_label": "y", "series": series}


def timeline_plot(entries, node, event, field):
    segs, cur = [], None
    rel_end = max((e["_rel"] for e in entries), default=0)
    for e in entries:
        if e.get("node") != node or e.get("event") != event or field not in e:
            continue
        if cur is None or e[field] != cur["label"]:
            if cur:
                cur["end"] = e["_rel"]
            cur = {"name": f"{node}.{field}", "label": str(e[field]), "start": e["_rel"], "end": rel_end}
            segs.append(cur)
    return {"title": f"{node} / {event}.{field}", "kind": "timeline", "x_label": "time [s]", "segments": segs}
