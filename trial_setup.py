"""試行の条件、送信側の起動、結果ファイルの保存。"""
import argparse
import csv
import json
import subprocess
import sys
from contextlib import contextmanager
from pathlib import Path

from action_msgs.msg import GoalStatus

from fault_stream import PATTERNS


def parse_options():
    parser = argparse.ArgumentParser(description="肩関節の指令途絶実験")
    parser.add_argument("mode", choices=["baseline", "guard", "continuous", "timing", "fault"])
    parser.add_argument("output", type=Path)
    parser.add_argument("--timeout", type=float, default=0.30)
    parser.add_argument("--send-stop", action="store_true", help="guardで最後の指令と同時にSTOPを送る")
    parser.add_argument("--resume-gap", type=float, help="timingで20通目と21通目の間に空ける秒数")
    parser.add_argument("--pattern", choices=PATTERNS)
    parser.add_argument("--policy", choices=["arrival", "ordered", "issued"])
    args = parser.parse_args()
    if args.mode == "fault":
        if not (args.pattern and args.policy):
            parser.error("faultには --pattern と --policy が必要です")
    elif args.pattern or args.policy:
        parser.error("--pattern / --policy はfault用です")
    if args.send_stop and args.mode != "guard":
        parser.error("--send-stop はguard用です")
    if args.resume_gap is not None and (args.mode != "timing" or not 0.10 <= args.resume_gap <= 1):
        parser.error("--resume-gap はtiming用です（0.10〜1秒）")
    if not 0.10 < args.timeout < 3:
        parser.error("--timeout は0.10秒より長く3秒未満です")
    args.output.mkdir(parents=True, exist_ok=False)
    return args


def command_count(options):
    return 85 if options.mode == "continuous" or options.resume_gap is not None else 20


def observation_seconds(options):
    if options.mode == "fault":
        return 7.4
    if command_count(options) == 85:
        return 9.0 + (options.resume_gap or 0.0)
    return 7.3


@contextmanager
def run_operator(options):
    if options.mode == "fault":
        script = "fault_stream.py"
        args = [options.pattern, str(options.output / "sender.csv")]
    else:
        script = "operator_stream.py"
        args = ["--count", str(command_count(options))]
        if options.send_stop:
            args.append("--send-stop")
        if options.resume_gap is not None:
            args.extend(["--resume-gap", str(options.resume_gap)])
    sender = subprocess.Popen([sys.executable, str(Path(__file__).with_name(script)), *args])
    try:
        yield sender
    finally:
        if sender.poll() is None:
            sender.terminate()
            sender.wait(timeout=4)


def save_results(options, receiver, samples, initial_angle, status, sender_status):
    events = receiver.events
    count = len(receiver.arrivals)
    if sender_status or (options.mode != "fault" and count != command_count(options)):
        raise RuntimeError("送受信した指令数を確認できません")
    cancelled = "cancel_requested" in events
    should_cancel = {"guard": True, "baseline": False, "continuous": False}.get(options.mode, cancelled)
    expected = GoalStatus.STATUS_CANCELED if should_cancel else GoalStatus.STATUS_SUCCEEDED
    if cancelled != should_cancel or status != expected:
        raise RuntimeError(f"{options.mode} の指令と腕の終了状態が一致しません（status={status}）")
    events.update(mode=options.mode, timeout_s=options.timeout, initial_rad=initial_angle,
                  goal_rad=-0.90, duration_s=6, action_status=status, heartbeats=receiver.arrivals)
    if options.mode == "timing":
        events.update(resume_gap_s=options.resume_gap, reference_command=20)
    if options.mode == "fault":
        events.update(pattern=options.pattern, policy=options.policy, permits=receiver.permits)
    (options.output / "events.json").write_text(json.dumps(events, indent=2) + "\n")
    with (options.output / "states.csv").open("x", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sim_s", "monotonic_s", "shoulder_rad"])
        writer.writerows(samples)
    print(options.mode, options.output, count, status, flush=True)
