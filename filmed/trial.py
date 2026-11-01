"""Single simulated operator-link trial. Run only in the isolated Gazebo environment."""
import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

NAMES = [f"crane_plus_joint{i}" for i in range(1, 5)]


def goal(angle, duration):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = NAMES
    g.trajectory.points = [JointTrajectoryPoint(
        positions=[0.0, angle, 0.0, 0.0], time_from_start=Duration(sec=duration))]
    return g


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["baseline", "guard", "continuous"])
    p.add_argument("output", type=Path)
    p.add_argument("--timeout", type=float, default=0.30)
    args = p.parse_args()
    if not 0.10 < args.timeout < 3:
        p.error("Use a timeout longer than one command period and shorter than 3 s")
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("crane_sim_receiver", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    client = ActionClient(node, FollowJointTrajectory,
                          "/crane_plus_arm_controller/follow_joint_trajectory")
    samples, beats, events = [], [], {}
    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def state(msg):
        if NAMES[1] in msg.name:
            stamp = msg.header.stamp
            samples.append((stamp.sec + stamp.nanosec * 1e-9, time.monotonic(),
                            msg.position[msg.name.index(NAMES[1])]))

    def until(condition, seconds, description):
        deadline = time.monotonic() + seconds
        while not condition():
            if time.monotonic() > deadline:
                raise RuntimeError(description)
            rclpy.spin_once(node, timeout_sec=0.02)

    node.create_subscription(JointState, "/joint_states", state, qos_profile_sensor_data)
    until(lambda: len(samples) > 2 and now() > 0, 15, "No simulation feedback")
    if not client.wait_for_server(timeout_sec=12):
        raise RuntimeError("No arm controller")
    reset_req = client.send_goal_async(goal(0.0, 2))
    until(reset_req.done, 5, "Reset did not start")
    reset = reset_req.result()
    if not reset.accepted:
        raise RuntimeError("Reset rejected")
    done = reset.get_result_async()
    until(done.done, 25, "Reset timeout")
    if done.result().status != 4:
        raise RuntimeError("Reset failed")
    settle = now() + 0.45
    until(lambda: now() >= settle and abs(samples[-1][2]) < 0.004, 10, "Reset pose mismatch")
    initial = samples[-1][2]
    request = None

    def operator(msg):
        nonlocal request
        if abs(msg.data + 0.9) > 1e-6:
            return
        beats.append((now(), time.monotonic()))
        if request is None:
            request = client.send_goal_async(goal(msg.data, 6))

    node.create_subscription(Float64, "/crane_demo/operator_target", operator, 10)
    count = 85 if args.mode == "continuous" else 20
    process = subprocess.Popen([sys.executable, str(Path(__file__).with_name("operator_stream.py")),
                                "--count", str(count)])
    handle = None
    result = None
    cancel = None
    try:
        until(lambda: request is not None and request.done(), 20, "Missing operator command or action")
        handle = request.result()
        if not handle.accepted:
            raise RuntimeError("Motion rejected")
        result = handle.get_result_async()
        end = beats[0][0] + (9.0 if args.mode == "continuous" else 7.3)
        deadline = time.monotonic() + 80
        while now() < end:
            if time.monotonic() > deadline:
                raise RuntimeError("Trial timed out")
            rclpy.spin_once(node, timeout_sec=0.02)
            if (args.mode != "baseline" and cancel is None and not result.done()
                    and now() - beats[-1][0] >= args.timeout):
                events["cancel_requested"] = now()
                cancel = handle.cancel_goal_async()
        until(result.done, 3, "Missing end-of-motion result")
        if cancel is not None:
            until(cancel.done, 3, "Cancel request not acknowledged")
            if not cancel.result().goals_canceling:
                raise RuntimeError("Controller rejected cancellation")
        process.wait(timeout=8)
        if process.returncode or len(beats) != count:
            raise RuntimeError(f"Sent/received command count mismatch: {len(beats)}/{count}")
        outcome = result.result()
        expected_status = 5 if args.mode == "guard" else 4
        if outcome.status != expected_status or ('cancel_requested' in events) != (args.mode == 'guard'):
            raise RuntimeError(f"Unexpected action outcome {outcome.status}, {events}")
        events.update(mode=args.mode, timeout_s=args.timeout, initial_rad=initial,
                      goal_rad=-0.9, duration_s=6, action_status=outcome.status,
                      heartbeats=beats)
        (out / "events.json").write_text(json.dumps(events, indent=2) + "\n")
        with (out / "states.csv").open("x", newline="") as f:
            w = csv.writer(f)
            w.writerow(["sim_s", "monotonic_s", "shoulder_rad"])
            w.writerows(samples)
        print(args.mode, out, len(beats), outcome.status, flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=4)
        if handle is not None and result is not None and not result.done():
            pending = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, pending, timeout_sec=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
