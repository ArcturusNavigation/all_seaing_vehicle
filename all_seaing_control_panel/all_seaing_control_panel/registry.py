"""
The registry of everything the control panel can run: launch files, scripts, analysis
tools, recordings... defined in config/registry.yaml (see the comments there for the format).
Adding a new tool to the panel = adding an entry there, no code changes.

Each action becomes a shell command on its target:
- kind "launch":  ros2 launch <package> <file> name:=value ...   (only args with a value)
- kind "run":     ros2 run <package> <executable> <rendered args>
- kind "command": the `command` template, with {name} replaced by the shell-quoted value and
  {path_<key>} by the target's configured path (repo, workspace, logs, bags, panel; see targets.yaml)

Argument types: string, number, bool, choice (`choices`), and dynamic choices filled from
the target: bag (folders in the bags path), run (folders in the run-logs path).
An action's `env` ({NAME: value}) is set for its command (e.g. ROS_LOCALHOST_ONLY: "1").
For kind run/command, an arg is rendered with its `flag` (e.g. "-i {value}", "--loop" for
bools) or, without a flag, the value alone.
"""

import shlex

import yaml


class Registry:
    def __init__(self, path):
        with open(path) as f:
            cfg = yaml.safe_load(f)
        self.categories = cfg.get("categories", [])
        self.actions = {a["id"]: a for a in cfg["actions"]}

    def public(self):
        return {"categories": self.categories, "actions": list(self.actions.values())}

    @staticmethod
    def _value(arg, values):
        v = values.get(arg["name"], arg.get("default", ""))
        if arg.get("type") == "bool":
            v = str(v).lower() in ("true", "1", "yes", "on")
        return v

    @staticmethod
    def _path_subs(paths):
        """{path_<key>} placeholders from the target's paths (~ expanded by the target's shell)."""
        subs = {}
        for k, v in (paths or {}).items():
            v = str(v)
            subs[f"path_{k}"] = '"$HOME' + v[1:] + '"' if v.startswith("~") else shlex.quote(v)
        return subs

    def build_command(self, action_id, values, paths=None):
        """The action's shell command, prefixed with its `env` (e.g. ROS_LOCALHOST_ONLY=1) if any."""
        env = self.actions[action_id].get("env") or {}
        cmd = self._build_command(action_id, values, paths)
        if env:
            cmd = " ".join(f"{k}={shlex.quote(str(v))}" for k, v in env.items()) + " " + cmd
        return cmd

    def _build_command(self, action_id, values, paths=None):
        a = self.actions[action_id]
        kind = a.get("kind", "command")
        args = a.get("args", [])
        if kind == "launch":
            parts = ["ros2", "launch", a["package"], a["file"]]
            for arg in args:
                v = self._value(arg, values)
                if isinstance(v, bool):
                    v = "true" if v else "false"
                if v != "" and v is not None:
                    parts.append(f"{arg['name']}:={v}")
            return " ".join(shlex.quote(p) for p in parts)
        if kind == "run":
            parts = ["ros2", "run", a["package"], a["executable"]]
            rendered = []
            for arg in args:
                v = self._value(arg, values)
                flag = arg.get("flag")
                if isinstance(v, bool):
                    if v and flag:
                        rendered += shlex.split(flag)  # e.g. "--ros-args -p use_sim_time:=true"
                    continue
                if v == "" or v is None:
                    continue
                if flag and "{value}" in flag:
                    rendered += [f.replace("{value}", str(v)) for f in shlex.split(flag)]
                elif flag:
                    rendered += [flag, str(v)]
                else:
                    rendered += shlex.split(str(v)) if arg.get("split") else [str(v)]
            return " ".join(shlex.quote(p) for p in parts + rendered)
        # kind command: template with {name} (args) and {path_<key>} (target paths) placeholders
        subs = self._path_subs(paths)
        for arg in args:
            v = self._value(arg, values)
            subs[arg["name"]] = ("true" if v else "false") if isinstance(v, bool) else shlex.quote(str(v))
        return a["command"].format(**subs)
