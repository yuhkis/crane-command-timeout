"""Issue stamped motion permits and inject controlled delivery faults in Gazebo."""
import argparse
import csv
import heapq
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Header

PATTERNS = ("normal", "scattered", "burst", "replay", "queue_tail", "lag_loss")
CUTOFF = ("replay", "queue_tail")
MS = 1_000_000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pattern", choices=PATTERNS)
    parser.add_argument("sender_log", type=Path)
    args = parser.parse_args()
    count = 20 if args.pattern in CUTOFF else 70
    drops = {"scattered": {14, 24, 34, 44}, "burst": {20, 21, 22, 23}, "lag_loss": {21}}.get(args.pattern, set())
    rclpy.init()
    node = rclpy.create_node("crane_fault_injector", parameter_overrides=[Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Header, "/crane_demo/motion_permit", 10)
    clock = node.get_clock()
    try:
        deadline = time.monotonic() + 20
        while not clock.now().nanoseconds or not pub.get_subscription_count():
            if time.monotonic() >= deadline:
                raise RuntimeError("Simulation clock or receiver missing")
            rclpy.spin_once(node, timeout_sec=.02)
        start = clock.now().nanoseconds + 100 * MS
        births = {}
        pending = []
        log = []
        seq = 1
        deadline = time.monotonic() + 80
        while seq <= count or pending:
            if time.monotonic() >= deadline:
                raise RuntimeError("Sender timeout")
            rclpy.spin_once(node, timeout_sec=.01)
            now = clock.now().nanoseconds
            if seq <= count and now >= start + (seq-1) * 100 * MS:
                births[seq] = now
                log.append(("generate", seq, now, now))
                if seq in drops:
                    log.append(("drop", seq, now, now))
                else:
                    delay = 180 if args.pattern == "lag_loss" else 0
                    if args.pattern == "queue_tail" and seq >= 16:
                        delay = (seq-15) * 45
                    heapq.heappush(pending, (now + delay * MS, seq, "send"))
                if args.pattern == "replay" and seq == 20:
                    for offset, old_seq in [(180,19), (360,20), (540,19), (720,20)]:
                        heapq.heappush(pending, (now + offset * MS, old_seq, "replay"))
                seq += 1
            while pending and pending[0][0] <= now:
                _, number, kind = heapq.heappop(pending)
                packet = Header()
                packet.stamp.sec, packet.stamp.nanosec = divmod(births[number], 1_000_000_000)
                pub.publish(packet)
                log.append((kind, number, births[number], now))
        with args.sender_log.open("x", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["event", "seq", "issued_ns", "event_ns"])
            writer.writerows(log)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
