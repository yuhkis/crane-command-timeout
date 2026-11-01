"""Gazeboとの接続、肩の軌道操作、関節角の記録。"""
import time
from contextlib import contextmanager

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f"crane_plus_joint{i}" for i in range(1, 5)]


def wait_until(node, ready, seconds, operation):
    deadline = time.monotonic() + seconds
    while not ready():
        if time.monotonic() >= deadline:
            raise TimeoutError(operation)
        rclpy.spin_once(node, timeout_sec=0.02)


def shoulder_goal(angle, seconds):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    goal.trajectory.points = [JointTrajectoryPoint(
        positions=[0.0, angle, 0.0, 0.0], time_from_start=Duration(sec=seconds))]
    return goal


class GazeboArm(Node):
    def __init__(self):
        super().__init__("crane_sim_receiver", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self, FollowJointTrajectory, "/crane_plus_arm_controller/follow_joint_trajectory")
        self.create_subscription(JointState, "/joint_states", self.record_angle, qos_profile_sensor_data)
        self.samples = []
        self.request = self.goal_handle = self.result = None
        self.cancel_sent = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record_angle(self, msg):
        if JOINTS[1] in msg.name:
            stamp = msg.header.stamp
            angle = msg.position[msg.name.index(JOINTS[1])]
            self.samples.append((stamp.sec + stamp.nanosec * 1e-9, time.monotonic(), angle))

    def wait_for_motion(self):
        wait_until(self, lambda: self.request is not None and self.request.done(), 20, "腕の軌道受付")
        self.goal_handle = self.request.result()
        if not self.goal_handle.accepted:
            raise RuntimeError("腕が軌道を受け付けませんでした")
        self.result = self.goal_handle.get_result_async()

    def finish_motion(self, seconds=3):
        wait_until(self, self.result.done, seconds, "腕の動作完了")
        return self.result.result().status

    def reset_pose(self):
        wait_until(self, self.client.server_is_ready, 12, "腕のコントローラに接続")
        wait_until(self, lambda: self.samples and self.now() > 0, 15, "関節角の受信")
        self.move(0.0, seconds=2)
        self.wait_for_motion()
        if self.finish_motion(seconds=25) != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("初期姿勢に戻れませんでした")
        settle = self.now() + 0.45
        wait_until(self, lambda: self.now() >= settle and abs(self.samples[-1][2]) < 0.004, 10, "肩の静止")
        self.request = self.goal_handle = self.result = None
        return self.samples[-1][2]

    def move(self, angle, seconds=6):
        self.request = self.client.send_goal_async(shoulder_goal(angle, seconds))

    def observe_until(self, end):
        deadline = time.monotonic() + 80
        while self.now() < end:
            if time.monotonic() >= deadline:
                raise TimeoutError("シミュレーション時計が進みません")
            rclpy.spin_once(self, timeout_sec=0.02)
            yield self.now()

    def is_moving(self):
        return self.result is not None and not self.result.done() and not self.cancel_sent

    def cancel(self):
        self.cancel_sent = True
        return self.goal_handle.cancel_goal_async()


@contextmanager
def run_gazebo():
    rclpy.init()
    arm = GazeboArm()
    try:
        yield arm
    finally:
        if arm.result is not None and not arm.result.done():
            rclpy.spin_until_future_complete(arm, arm.cancel(), timeout_sec=2)
        arm.destroy_node()
        rclpy.shutdown()
