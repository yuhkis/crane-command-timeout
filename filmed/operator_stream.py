"""Send a bounded series of identical movement commands in simulation time."""
import argparse
import time

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=20)
    args = parser.parse_args()
    if args.count < 1:
        parser.error("count must be positive")
    rclpy.init()
    node = rclpy.create_node("crane_sim_operator", parameter_overrides=[
        Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Float64, "/crane_demo/operator_target", 10)
    deadline = time.monotonic() + 15
    while not pub.get_subscription_count() or not node.get_clock().now().nanoseconds:
        if time.monotonic() >= deadline:
            raise RuntimeError("No receiver or simulation clock")
        rclpy.spin_once(node, timeout_sec=0.03)
    sent = 0

    def tick():
        nonlocal sent
        pub.publish(Float64(data=-0.90))
        sent += 1

    node.create_timer(0.10, tick)
    deadline = time.monotonic() + max(30, args.count)
    while sent < args.count:
        if time.monotonic() >= deadline:
            raise RuntimeError("Simulation clock stopped during transmission")
        rclpy.spin_once(node, timeout_sec=0.10)
    node.destroy_node()
    rclpy.shutdown()  # The publisher exits without sending any stop command.


if __name__ == "__main__":
    main()
