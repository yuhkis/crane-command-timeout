"""Run paired timeout comparisons in an isolated ROS/Gazebo environment."""
import argparse
import csv
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("Use a positive repeat count")
    args.output.mkdir(parents=True, exist_ok=False)
    scenarios = [("normal", 0.10), ("gap250", 0.25), ("gap350", 0.35), ("dropout", None)]
    plan = []
    for rep in range(args.repeats):
        start = rep % len(scenarios)
        for index, (name, gap) in enumerate(scenarios[start:] + scenarios[:start]):
            timeouts = [0.20, 0.30] if (rep + scenarios.index((name, gap))) % 2 == 0 else [0.30, 0.20]
            for timeout in timeouts:
                trial = f"r{rep+1}-{name}-t{round(timeout*1000)}"
                plan.append((len(plan)+1, rep+1, name, gap, timeout, trial))
    with (args.output / "plan.csv").open("x", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["order", "rep", "scenario", "gap_s", "timeout_s", "trial"])
        writer.writerows(plan)
    for order, _, _, gap, timeout, trial in plan:
        cmd = [sys.executable, str(Path(__file__).with_name("trial.py")), "timing",
               str(args.output / trial), "--timeout", str(timeout)]
        if gap is not None:
            cmd.extend(["--resume-gap", str(gap)])
        print(f"{order}/{len(plan)}  {trial}", flush=True)
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
