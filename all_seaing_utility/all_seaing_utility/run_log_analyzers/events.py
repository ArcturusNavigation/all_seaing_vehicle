"""Generic analyzer: how often each (node, event) pair occurs, with first/last time."""

NAME = "events"
DESCRIPTION = "count, rate and first/last time of every (node, event) pair"


def analyze(entries, args):
    groups = {}
    for e in entries:
        key = (e.get("node", "?"), e.get("event", "?"))
        g = groups.setdefault(key, [0, e["_rel"], e["_rel"]])
        g[0] += 1
        g[2] = e["_rel"]
    lines = [f"{'node':28} {'event':32} {'count':>7} {'rate[Hz]':>9} {'first[s]':>9} {'last[s]':>9}"]
    for (node, event), (count, first, last) in sorted(groups.items()):
        rate = (count - 1) / (last - first) if last > first else 0.0
        lines.append(f"{node:28} {event:32} {count:7d} {rate:9.2f} {first:9.2f} {last:9.2f}")
    return "\n".join(lines)


def summary(entries, args):
    """Rows {node, event, count, rate, first, last} for a table (control panel)."""
    groups = {}
    for e in entries:
        g = groups.setdefault((e.get("node", "?"), e.get("event", "?")), [0, e["_rel"], e["_rel"], e.get("level", "INFO")])
        g[0] += 1
        g[2] = e["_rel"]
    return [{"node": n, "event": ev, "count": c, "rate": round((c - 1) / (last - first), 2) if last > first else 0.0,
             "first": round(first, 2), "last": round(last, 2), "level": lvl}
            for (n, ev), (c, first, last, lvl) in sorted(groups.items())]
