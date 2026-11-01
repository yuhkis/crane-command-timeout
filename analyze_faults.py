"""Validate fault injection against the plan and summarize the observed arm motion."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
from statistics import median

from analyze import interp, joint_series, motion_after


def read_delivery(folder, plan, event):
    """計画した指令列を、実際の送受信ログと照合する。"""
    pattern = plan["pattern"]
    if (event["pattern"], event["policy"]) != (pattern, plan["policy"]):
        raise ValueError(f"{folder}: metadata does not match the plan")
    with (folder / "sender.csv").open() as f:
        sent_log = list(csv.DictReader(f))
    generated = [row for row in sent_log if row["event"] == "generate"]
    dropped = [int(row["seq"]) for row in sent_log if row["event"] == "drop"]
    sent_stamps = [int(row["issued_ns"]) for row in sent_log if row["event"] in ("send", "replay")]
    received_stamps = [stamp for _, stamp, _ in event["permits"]]
    if Counter(sent_stamps) != Counter(received_stamps):
        raise ValueError(f"{folder}: delivered permits do not reconcile with publications")

    count = 20 if pattern in ("queue_tail", "replay") else 70
    expected_drops = {"scattered": [14, 24, 34, 44], "burst": [20, 21, 22, 23], "lag_loss": [21]}
    sequence = [int(row["seq"]) for row in generated]
    stamp_by_sequence = {int(row["seq"]): int(row["issued_ns"]) for row in generated}
    if sequence != list(range(1, count + 1)) or dropped != expected_drops.get(pattern, []):
        raise ValueError(f"{folder}: injection schedule differs from design")
    if len(set(stamp_by_sequence.values())) != count:
        raise ValueError(f"{folder}: duplicate generation timestamp")
    replays = sum(row["event"] == "replay" for row in sent_log)
    return stamp_by_sequence, len(dropped), replays


def read_trial(folder, plan):
    event = json.loads((folder / "events.json").read_text())
    stamp_by_sequence, dropped, replays = read_delivery(folder, plan, event)
    pattern = plan["pattern"]
    rx = event["permits"]
    t, q = joint_series(folder)
    arrival = [at for at, _, _ in rx]
    age = [at - stamp * 1e-9 for at, stamp, _ in rx]
    cancelled = "cancel_requested" in event
    if event["action_status"] != (5 if cancelled else 4):
        raise ValueError(f"{folder}: unexpected terminal status {event['action_status']}")
    row = {
        **plan,
        "generated": len(stamp_by_sequence),
        "dropped": dropped,
        "received": len(rx),
        "replay_transmissions": replays,
        "max_rx_gap_s": max(b - a for a, b in zip(arrival, arrival[1:])),
        "median_age_s": median(age),
        "max_age_s": max(age),
        "old_sequence": sum(decision == "old_sequence" for _, _, decision in rx),
        "invalid_age": sum(decision == "invalid_age" for _, _, decision in rx),
        "cancelled": int(cancelled),
        "at_goal": int(abs(q[-1] - event["goal_rad"]) <= 0.01),
        "final_rad": q[-1],
        "cancel_after_source_end_s": None,
        "stop_after_source_end_s": None,
        "post_end_travel_deg": None,
        "pre_end_speed_rad_s": None,
        "post_recovery_deg": None,
    }
    if pattern in ("queue_tail", "replay"):
        last_generated = stamp_by_sequence[len(stamp_by_sequence)] * 1e-9
        motion = motion_after(t, q, last_generated)
        row["stop_after_source_end_s"] = motion["stop_s"]
        row["post_end_travel_deg"] = motion["post_travel_deg"]
        row["pre_end_speed_rad_s"] = abs(
            interp(t, q, last_generated - 0.05) - interp(t, q, last_generated - 0.35)
        ) / 0.30
        if cancelled:
            row["cancel_after_source_end_s"] = event["cancel_requested"] - last_generated
    if pattern == "burst":
        recovery = next(at for at, stamp, _ in rx if stamp == stamp_by_sequence[24])
        position = interp(t, q, recovery)
        row["post_recovery_deg"] = max(
            abs(math.degrees(angle - position)) for at, angle in zip(t, q) if at >= recovery
        )
    return row


def summarize_group(pattern, policy, trials):
    if len(trials) != 3:
        raise ValueError(f"Incomplete group: {pattern}, {policy}")
    item = {
        "pattern": pattern,
        "policy": policy,
        "n": len(trials),
        "cancelled": sum(row["cancelled"] for row in trials),
        "at_goal": sum(row["at_goal"] for row in trials),
    }
    for key in ("max_rx_gap_s", "max_age_s", "stop_after_source_end_s", "post_end_travel_deg",
                "cancel_after_source_end_s", "pre_end_speed_rad_s", "post_recovery_deg", "old_sequence"):
        values = [row[key] for row in trials if row[key] is not None]
        if values:
            item[key] = {"min": min(values), "median": median(values), "max": max(values)}
    return item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    root = parser.parse_args().root
    with (root / "plan.csv").open() as f:
        plan = list(csv.DictReader(f))
    if not plan or len({entry["trial"] for entry in plan}) != len(plan):
        raise ValueError("Empty plan or duplicate trials")
    rows = [read_trial(root / entry["trial"], entry) for entry in plan]
    with (root / "measurements.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    groups = defaultdict(list)
    for row in rows:
        groups[(row["pattern"], row["policy"])].append(row)
    summary = [summarize_group(pattern, policy, trials) for (pattern, policy), trials in sorted(groups.items())]
    (root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"{len(rows)} trials, {len(groups)} pattern/policy combinations")


if __name__ == "__main__":
    main()
