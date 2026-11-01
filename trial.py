"""Gazeboの肩を動かし、操作指令が途絶えたときの動きを記録する。"""
import time

from std_msgs.msg import Empty, Float64, Header

from gazebo_arm import GazeboArm, run_gazebo
from trial_setup import parse_options, run_operator, observation_seconds, save_results


def watch_motion(arm, receiver, options):
    """最後の有効な指令からの経過時間を調べ、必要なら軌道を取り消す。"""
    end = receiver.arrivals[0][0] + observation_seconds(options)
    for now in arm.observe_until(end):
        if options.mode == "baseline" or not arm.is_moving():
            continue
        if "stop_received" in receiver.events:
            reason = "stop"
        elif now - receiver.last_valid >= options.timeout:
            reason = "timeout"
        else:
            continue
        arm.cancel()
        receiver.events.update(cancel_reason=reason, cancel_requested=now)


def run_trial(arm: GazeboArm, options):
    initial_angle = arm.reset_pose()
    receiver = Receiver(arm, options)
    with run_operator(options) as sender:
        arm.wait_for_motion()
        watch_motion(arm, receiver, options)
        status = arm.finish_motion()
        sender.wait(timeout=8)
        save_results(options, receiver, arm.samples, initial_angle, status, sender.returncode)


class Receiver:
    def __init__(self, arm, options):
        self.arm = arm
        self.options = options
        self.last_valid = None
        self.last_stamp_ns = None
        self.arrivals = []
        self.permits = []
        self.events = {}

        if options.mode == "fault":
            arm.create_subscription(Header, "/crane_demo/motion_permit", self.on_permit, 10)
        else:
            arm.create_subscription(Float64, "/crane_demo/operator_target", self.on_target, 10)
        if options.mode == "guard":
            arm.create_subscription(Empty, "/crane_demo/operator_stop", self.on_stop, 10)
            self.events["stop_planned"] = options.send_stop

    def on_target(self, message):
        if abs(message.data + 0.90) <= 1e-6:
            received = self.arm.now()
            self.arrivals.append((received, time.monotonic()))
            self.allow_motion(received, message.data)

    def allow_motion(self, valid_from, angle):
        # 最初の許可で一度だけ軌道を送る。以後は有効時刻だけを更新する。
        if self.last_valid is None:
            self.arm.move(angle)
        self.last_valid = valid_from

    def on_stop(self, _message):
        self.events.setdefault("stop_received", self.arm.now())

    def on_permit(self, message):
        """追加実験では、再送や遅延がある指令を方式ごとに選別する。"""
        received = self.arm.now()
        stamp_ns = message.stamp.sec * 1_000_000_000 + message.stamp.nanosec
        issued = stamp_ns * 1e-9
        policy = self.options.policy
        decision = "accepted"
        if policy != "arrival" and self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            decision = "old_sequence"
        elif policy == "issued" and (received - issued >= self.options.timeout or issued > received + 0.010):
            decision = "invalid_age"

        self.arrivals.append((received, time.monotonic()))
        self.permits.append((received, stamp_ns, decision))
        if decision == "accepted":
            self.last_stamp_ns = stamp_ns
            valid_from = min(issued, received) if policy == "issued" else received
            self.allow_motion(valid_from, -0.90)


if __name__ == "__main__":
    options = parse_options()
    with run_gazebo() as arm:
        run_trial(arm, options)
