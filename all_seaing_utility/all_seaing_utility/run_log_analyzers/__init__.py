"""
Node/task-specific analyzers for run_log_analysis.py. Each module here defines:
    NAME = "<name>"; DESCRIPTION = "<one line>"; def analyze(entries, args) -> str
where `entries` is the merged, time-sorted (and already filtered) list of log entries
(dicts with "node", "level", "event", "_rel" = seconds from run start, and the logged fields).
"""
