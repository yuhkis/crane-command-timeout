"""Run three counterbalanced blocks of six traffic patterns and two receiver policies."""
import argparse
import csv
from pathlib import Path
import subprocess
import sys

from fault_stream import PATTERNS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path)
    parser.add_argument("--ordered-only", action="store_true",
                        help="Run the sequence-only follow-up arm in a separate directory")
    args = parser.parse_args()
    args.results.mkdir(parents=True, exist_ok=False)
    plan = []
    for block in range(3):
        for i in range(len(PATTERNS)):
            index = (i+2*block) % len(PATTERNS)
            pattern = PATTERNS[index]
            policies = ("arrival", "issued") if (index+block)%2 == 0 else ("issued", "arrival")
            if args.ordered_only:
                policies = ("ordered",)
            for policy in policies:
                trial = f"r{block+1}-{pattern}-{policy}"
                plan.append((len(plan)+1,block+1,pattern,policy,trial))
    with (args.results/"plan.csv").open("x",newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["order","block","pattern","policy","trial"])
        writer.writerows(plan)
    for n, _, pattern, policy, trial in plan:
        print(f"{n}/{len(plan)} {trial}", flush=True)
        subprocess.run([sys.executable,str(Path(__file__).with_name("trial.py")),"fault",
                        str(args.results/trial),"--policy",policy,"--pattern",pattern],check=True)


if __name__ == "__main__":
    main()
