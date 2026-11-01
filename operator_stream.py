"""Send a bounded series of identical movement commands in simulation time."""
import argparse

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, Float64

from gazebo_arm import wait_until


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--send-stop", action="store_true",
                        help="Publish one explicit STOP with the last movement command")
    parser.add_argument("--resume-gap", type=float,
                        help="Interval from command 20 to 21, in simulation seconds")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("count must be positive")
    if args.resume_gap is not None and (args.count < 21 or not 0.10 <= args.resume_gap <= 1):
        parser.error("A resumed stream needs at least 21 commands and a gap of 0.10 to 1.00 s")
    rclpy.init()
    node = rclpy.create_node("crane_sim_operator", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Float64, "/crane_demo/operator_target", 10)
    stop_pub = node.create_publisher(Empty, "/crane_demo/operator_stop", 10) if args.send_stop else None

    def receiver_ready():
        clock_ready = node.get_clock().now().nanoseconds > 0
        stop_ready = stop_pub is None or stop_pub.get_subscription_count() > 0
        return clock_ready and pub.get_subscription_count() > 0 and stop_ready

    wait_until(node, receiver_ready, 15, "操作指令の受信側に接続")
    sent = 0

    def send_target():
        nonlocal sent
        pub.publish(Float64(data=-0.90))
        sent += 1
        if sent == args.count and stop_pub is not None:
            stop_pub.publish(Empty())
        if args.resume_gap not in (None, 0.10) and sent in (20, 21):
            period = args.resume_gap if sent == 20 else 0.10
            timer.timer_period_ns = round(period * 1e9)
            timer.reset()

    timer = node.create_timer(0.10, send_target)
    wait_until(node, lambda: sent >= args.count, max(30, args.count), "操作指令の送信完了")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
