"""Measure post-command motion in simulation time from ROS joint feedback."""
import argparse
import csv
import json
import math
import statistics
from bisect import bisect_left
from pathlib import Path


def interp(t, q, x):
    i = bisect_left(t, x)
    if i >= len(t) or (not i and x != t[0]):
        raise ValueError("Interpolation outside recorded times")
    if t[i] == x:
        return q[i]
    return q[i - 1] + (q[i] - q[i-1]) * (x-t[i-1]) / (t[i]-t[i-1])


def joint_series(path):
    with (path / "states.csv").open() as f:
        states = list(csv.DictReader(f))
    t = [float(x["sim_s"]) for x in states]
    q = [float(x["shoulder_rad"]) for x in states]
    if not t or not all(math.isfinite(v) for v in t + q):
        raise ValueError("Missing or non-finite joint data")
    if not all(right >= left for left, right in zip(t, t[1:])):
        raise ValueError("Joint timestamps are out of order")
    return t, q


def motion_after(t, q, last):
    zero = interp(t, q, last)
    sel = [i for i, ti in enumerate(t) if last <= ti <= last+5.1]
    if not sel or t[sel[-1]] < last+5.0 or max(t[i]-t[i-1] for i in sel) > 0.06:
        raise ValueError("Insufficient joint-state coverage")
    stable = None
    stopped = None
    for i in sel:
        speed = abs(q[i] - interp(t, q, t[i]-.1)) / .1
        if speed > .01:
            stable = None
        elif stable is None:
            stable = t[i]
        elif t[i]-stable >= .30 and stopped is None:
            stopped = stable
    if stopped is None:
        raise ValueError("No sustained stop")
    return {"cut_rad": zero, "post_travel_deg": math.degrees(max(abs(q[i]-zero) for i in sel)),
            "stop_s": stopped-last, "end_rad": q[sel[-1]]}


def summarize(path):
    e = json.loads((path / "events.json").read_text())
    if e["mode"] == "fault":
        raise ValueError("Use analyze_faults.py for fault-injection data")
    if e["mode"] == "timing":
        raise ValueError("Use analyze_timing.py for timing-study data")
    t, q = joint_series(path)
    beats = [b[0] for b in e["heartbeats"]]
    first, last = beats[0], beats[-1]
    worst_gap = max(b-a for a,b in zip(beats, beats[1:]))
    if worst_gap >= e["timeout_s"]:
        raise ValueError("Unexpected interruption before planned cutoff")
    if e["mode"] == "continuous":
        return {"mode": "continuous", "action_status": e["action_status"], "commands": len(beats),
                "cancelled": "cancel_requested" in e, "final_rad": q[-1],
                "max_command_gap_s": worst_gap}
    motion = motion_after(t, q, last)
    return {
        "trial": path.name, "mode": "explicit_stop" if e.get("stop_planned") else e["mode"], "commands": len(beats),
        "start_rad": e["initial_rad"], "rx_span_s": last-first,
        "precut_speed_rad_s": abs(interp(t,q,last-.05)-interp(t,q,last-.35))/.30,
        "cut_rad": motion["cut_rad"], "post_travel_deg": motion["post_travel_deg"],
        "stop_s": motion["stop_s"], "cancel_s": (e["cancel_requested"]-last if "cancel_requested" in e else None),
        "end_rad": motion["end_rad"], "max_command_gap_s": worst_gap,
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("results", type=Path)
    args = p.parse_args()
    rows = [summarize(f.parent) for f in sorted(args.results.glob("*/events.json"))]
    trials = [r for r in rows if r["mode"] != "continuous"]
    if not trials:
        p.error("No completed dropout trials")
    with (args.results/"measurements.csv").open("w", newline="") as f:
        w=csv.DictWriter(f, fieldnames=trials[0].keys())
        w.writeheader()
        w.writerows(trials)
    groups = {}
    for mode in ["baseline","guard","explicit_stop"]:
        rr=[r for r in trials if r["mode"] == mode]
        if not rr:
            continue
        groups[mode]={"n":len(rr)}
        for field in ["post_travel_deg","stop_s","precut_speed_rad_s"]:
            vs=[r[field] for r in rr]
            groups[mode][field]={"median":statistics.median(vs),"min":min(vs),"max":max(vs)}
    groups["continuous"]=[r for r in rows if r["mode"] == "continuous"]
    (args.results/"summary.json").write_text(json.dumps(groups,indent=2)+"\n")
    print(json.dumps(groups,indent=2))
