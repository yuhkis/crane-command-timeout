"""Compare receiver timing and shoulder motion with the prespecified trial plan."""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import median

from analyze import interp, joint_series, motion_after


def measure(root, plan):
    path = root / plan["trial"]
    event = json.loads((path / "events.json").read_text())
    timeout = float(plan["timeout_s"])
    gap = float(plan["gap_s"]) if plan["gap_s"] else None
    if (event["mode"], event["timeout_s"], event["resume_gap_s"], event["reference_command"]) != (
            "timing", timeout, gap, 20):
        raise ValueError(f"{path.name}: metadata disagrees with the plan")
    beats = [b[0] for b in event["heartbeats"]]
    intervals = [end - start for start, end in zip(beats, beats[1:])]
    if len(beats) != (20 if gap is None else 85) or not all(x > 0 for x in intervals):
        raise ValueError(f"{path.name}: missing or unordered commands")
    reference = beats[19]
    interval = intervals[19] if gap is not None else None
    ordinary = intervals[:19] + intervals[20:]
    t, q = joint_series(path)
    cancelled = "cancel_requested" in event
    cancel_delay = event["cancel_requested"] - reference if cancelled else None
    motion = motion_after(t, q, reference) if gap is None or cancelled else {}
    return {
        **plan, "commands": len(beats), "observed_gap_s": interval,
        "ordinary_min_s": min(ordinary), "ordinary_max_s": max(ordinary),
        "pre_event_speed_rad_s": abs(interp(t,q,reference-.05)-interp(t,q,reference-.35))/.30,
        "action_status": event["action_status"], "cancel_requested": cancelled,
        "cancel_before_resume": (cancelled and cancel_delay < interval) if interval is not None else None,
        "stop_relative_resume_s": motion["stop_s"] - interval if cancelled and interval is not None else None,
        "cancel_delay_s": cancel_delay, "final_rad": q[-1],
        "at_goal": abs(q[-1]-event["goal_rad"]) <= 0.01,
        "post_travel_deg": motion["post_travel_deg"] if gap is None else None,
        "stop_s": motion["stop_s"] if gap is None else None,
    }


def range_and_median(values):
    return {"min": min(values), "median": median(values), "max": max(values)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    args = parser.parse_args()
    root = args.results
    with (root / "plan.csv").open() as f:
        plan = list(csv.DictReader(f))
    if not plan or len({p["trial"] for p in plan}) != len(plan):
        parser.error("Plan empty or contains duplicate trial names")
    rows = [measure(root, p) for p in plan]
    with (root / "measurements.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    groups = defaultdict(list)
    for row in rows:
        groups[(row["scenario"], float(row["timeout_s"]))].append(row)
    summary = []
    for (scenario, timeout), trials in sorted(groups.items()):
        item = {"scenario": scenario, "timeout_s": timeout, "n": len(trials),
                "cancelled": sum(r["action_status"] == 5 for r in trials),
                "at_goal": sum(r["at_goal"] for r in trials)}
        for key in ("observed_gap_s", "cancel_delay_s", "stop_relative_resume_s", "post_travel_deg", "stop_s"):
            values = [r[key] for r in trials if r[key] is not None]
            if values:
                item[key] = range_and_median(values)
        summary.append(item)
    (root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Measured {len(rows)} trials across {len(summary)} conditions")


if __name__ == "__main__":
    main()
