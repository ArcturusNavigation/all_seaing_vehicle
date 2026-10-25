# Rules for AI coding agents

These rules apply to any AI agent working in this repository. They take precedence over default agent behaviour.

## 1. No unrequested changes
Don't write any code, and don't create, modify, or delete any files, unless the user has asked for it. Reading, searching, and running read-only commands to answer a question is fine. When a request is only a question ("why does X…", "how would we…"), answer it and propose changes, but don't make them.

## 2. Ask when something is unclear
If anything is unclear while implementing code or writing changes, stop and ask the user. That includes ambiguous requirements, more than one reasonable design, unknown parameter values or conventions, and anything that could behave differently in sim and on the real boat. Don't guess and don't fill gaps with assumptions.

## 3. Code quality and documentation
- **Neat and structured:** follow good coding practices and match the style, naming, and idioms of the surrounding code (see "Conventions" in `AGENTS.md`).
- **Don't reimplement existing code:** look for existing utilities before writing new ones and reuse them. Examples: `ActionServerBase`, `TaskServerBase`, `pid_controller.py`, `potential_field.py`, `buoy_utils.py`, `geometry_utils.py`, `perception_utilities.cpp`, `object_tracking_shared.cpp`.
- **Reusable and modular:** put shared logic in the appropriate common or utility module rather than copying it into individual nodes. Expose tunable values as ROS parameters or config YAML instead of hard-coding them.
- **Clear comments:** document what the code does, its specification (inputs and outputs, units, frames, topics, parameters, assumptions), and how to use it.
- **Keep the wiki up to date:** when a change adds or alters nodes, topics, parameters, launch files, or behaviour, update the corresponding page of the GitHub wiki (`all_seaing_vehicle.wiki` repo) as part of the same change. Add or adjust only what the change affects. Don't edit or rewrite existing content (past entries, unrelated sections, historical notes) unless the user asks.
