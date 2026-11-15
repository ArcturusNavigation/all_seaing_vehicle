"""
Parameters page (issue #327): view and change basic parameters (PID gains, thruster
positions and port assignments, GPS datum, frames, ...), defined in config/parameters.yaml.

Workflow:
1. Values are shown from the launch file (what the next launch uses) and, if the node is
   running, live (`ros2 param get`).
2. "Set" changes the running node (`ros2 param set`) and saves the value to the overrides
   file on the target (<repo>/all_seaing_bringup/config/control_panel_overrides.yaml).
   Many nodes only read their parameters at startup; those are marked `applies: restart`
   and only take effect after a relaunch.
3. "Apply to launch files" writes the saved overrides into the launch files (the `parameters=`
   literal of the matching Node), after showing a diff. Nothing is committed to git.

Locating a parameter in a launch file: the Node(...) call with executable="<executable>"
(and containing `match`, if given, to tell apart several nodes with the same executable),
then the `"<param>": <python literal>` entry inside it.
"""

import ast
import difflib
import re
import shlex

import yaml

OVERRIDES_REL = "all_seaing_bringup/config/control_panel_overrides.yaml"


def _node_blocks(source, executable):
    """(start, end) spans of launch_ros Node(...) calls whose executable is `executable`."""
    spans = []
    for m in re.finditer(r'executable\s*=\s*["\']' + re.escape(executable) + r'["\']', source):
        start = source.rfind("Node(", 0, m.start())
        if start < 0:
            continue
        depth, i = 0, start + len("Node")
        while i < len(source):
            c = source[i]
            if c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    spans.append((start, i + 1))
                    break
            elif c in "\"'":  # skip string literals (may contain parentheses)
                q = c
                i += 1
                while i < len(source) and source[i] != q:
                    i += 2 if source[i] == "\\" else 1
            elif c == "#":  # skip comments
                i = source.find("\n", i)
                if i < 0:
                    break
            i += 1
    return spans


def _literal_span(source, pos):
    """End index of the Python literal (number, string, bool, list) starting at pos."""
    c = source[pos]
    if c == "[":
        depth = 0
        for i in range(pos, len(source)):
            if source[i] == "[":
                depth += 1
            elif source[i] == "]":
                depth -= 1
                if depth == 0:
                    return i + 1
    if c in "\"'":
        return source.index(c, pos + 1) + 1
    m = re.match(r"[-+0-9.eE]+|True|False|None|-?np\.pi(?:\s*/\s*[0-9.]+)?", source[pos:])
    return pos + m.end() if m else pos


def find_param(source, executable, param, match=None):
    """(value_start, value_end) of the param's literal in the launch file, or None."""
    for start, end in _node_blocks(source, executable):
        block = source[start:end]
        if match and match not in block:
            continue
        for m in re.finditer(r'["\']' + re.escape(param) + r'["\']\s*:\s*', block):
            line_start = block.rfind("\n", 0, m.start()) + 1
            if "#" in block[line_start:m.start()]:
                continue  # commented-out entry, e.g. `# {"yaw_offset": -np.pi/2.0},`
            vs = start + m.end()
            return vs, _literal_span(source, vs)
    return None


def format_literal(value):
    if isinstance(value, bool):
        return "True" if value else "False"
    if isinstance(value, str):
        return repr(value).replace("'", '"')
    if isinstance(value, list):
        return "[" + ", ".join(format_literal(v) for v in value) + "]"
    return repr(value)


def parse_value(text, ptype):
    """Parse a value typed in the UI according to the parameter type."""
    if ptype == "bool":
        return str(text).strip().lower() in ("true", "1", "yes", "on")
    if ptype == "int":
        return int(text)
    if ptype == "double":
        return float(text)
    if ptype in ("double_array", "int_array"):
        vals = ast.literal_eval(text) if isinstance(text, str) else text
        return [float(v) if ptype == "double_array" else int(v) for v in vals]
    return str(text)


class Params:
    def __init__(self, cfg_path, targets):
        with open(cfg_path) as f:
            self.defs = yaml.safe_load(f)["parameters"]
        self.by_id = {p["id"]: p for p in self.defs}
        self.targets = targets

    def _overrides_path(self, t):
        return f"{t.path('repo')}/{OVERRIDES_REL}"

    def load_overrides(self, t):
        data, _ = t.read_file(self._overrides_path(t))
        return yaml.safe_load(data.decode()) or {} if data else {}

    def _launch_source(self, t, rel):
        data, _ = t.read_file(f"{t.path('repo')}/{rel}", max_bytes=5_000_000)
        return data.decode()

    def values(self, target_name):
        """All parameter definitions with launch-file, saved-override and live values."""
        t = self.targets[target_name]
        overrides = self.load_overrides(t)
        sources = {}
        # live values: one remote command for all parameters
        script = "; ".join(
            f"echo '@@{p['id']}'; timeout 4 ros2 param get {shlex.quote(p['node'])} {shlex.quote(p['param'])} 2>&1 | head -3"
            for p in self.defs)
        rc, out, _ = t.run(script, timeout=10 + 5 * len(self.defs))
        live = {}
        for chunk in out.split("@@")[1:]:
            pid, _, text = chunk.partition("\n")
            text = text.strip()
            live[pid.strip()] = text.split("value is:", 1)[1].strip() if "value is:" in text else None
        result = []
        for p in self.defs:
            rel = p["launch_file"]
            if rel not in sources:
                try:
                    sources[rel] = self._launch_source(t, rel)
                except Exception:
                    sources[rel] = ""
            span = find_param(sources[rel], p["executable"], p["param"], p.get("match"))
            launch_val = sources[rel][span[0]:span[1]] if span else None
            result.append({**p, "launch_value": launch_val, "live_value": live.get(p["id"]),
                           "saved_value": overrides.get(p["id"], {}).get("value")})
        return result

    def set(self, target_name, param_id, text, apply_live=True):
        """Save an override and (optionally) set it on the running node."""
        t = self.targets[target_name]
        p = self.by_id[param_id]
        value = parse_value(text, p["type"])
        overrides = self.load_overrides(t)
        overrides[param_id] = {"node": p["node"], "param": p["param"], "value": value,
                               "launch_file": p["launch_file"], "executable": p["executable"]}
        if p.get("match"):
            overrides[param_id]["match"] = p["match"]
        t.write_file(self._overrides_path(t), "# Written by the Arcturus control panel (Parameters page).\n"
                                              "# 'Apply to launch files' writes these values into the launch files.\n"
                     + yaml.safe_dump(overrides, sort_keys=True))
        live = None
        if apply_live:
            yaml_value = yaml.safe_dump(value, default_flow_style=True).strip().removesuffix("...").strip()
            rc, out, err = t.run(f"timeout 6 ros2 param set {shlex.quote(p['node'])} {shlex.quote(p['param'])} "
                                 f"{shlex.quote(yaml_value)}", timeout=12)
            live = (out + err).strip() or ("ok" if rc == 0 else "failed")
        return {"saved": value, "live_result": live}

    def clear(self, target_name, param_id):
        t = self.targets[target_name]
        overrides = self.load_overrides(t)
        overrides.pop(param_id, None)
        t.write_file(self._overrides_path(t), yaml.safe_dump(overrides, sort_keys=True))

    def plan_apply(self, target_name):
        """New contents + unified diffs of the launch files with the saved overrides applied."""
        t = self.targets[target_name]
        overrides = self.load_overrides(t)
        files, problems = {}, []
        for pid, o in overrides.items():
            rel = o["launch_file"]
            if rel not in files:
                files[rel] = [self._launch_source(t, rel)] * 2  # [original, modified]
            src = files[rel][1]
            span = find_param(src, o["executable"], o["param"], o.get("match"))
            if span is None:
                problems.append(f"{pid}: \"{o['param']}\" not found in the {o['executable']} node of {rel}")
                continue
            files[rel][1] = src[:span[0]] + format_literal(o["value"]) + src[span[1]:]
        diffs = {}
        for rel, (orig, new) in files.items():
            if orig != new:
                diffs[rel] = "".join(difflib.unified_diff(orig.splitlines(True), new.splitlines(True),
                                                          f"a/{rel}", f"b/{rel}"))
        return {"diffs": diffs, "problems": problems, "_new": {rel: v[1] for rel, v in files.items()}}

    def apply(self, target_name):
        plan = self.plan_apply(target_name)
        t = self.targets[target_name]
        for rel in plan["diffs"]:
            t.write_file(f"{t.path('repo')}/{rel}", plan["_new"][rel])
        return {"written": list(plan["diffs"]), "problems": plan["problems"]}
