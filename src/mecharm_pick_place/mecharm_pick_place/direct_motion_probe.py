"""Collision-planner-free smooth motion probe for transport validation."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


JOINT_NAMES = (
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "gripper_controller",
)


class DirectMotionProbe(Node):
    def __init__(self, node_name="direct_motion_probe"):
        super().__init__(node_name)
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            JointState, "/mecharm/joint_target", 10
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._state_callback, state_qos
        )
        self.positions = None

    def _state_callback(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            self.positions = tuple(float(values[name]) for name in JOINT_NAMES)

    def publish_positions(self, positions):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(positions)
        self.publisher.publish(message)


def _smoothstep(value):
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


def _execute_segment(node, start, target, duration=3.0, rate_hz=120.0):
    started = time.monotonic()
    period = 1.0 / rate_hz
    while rclpy.ok():
        elapsed = time.monotonic() - started
        ratio = _smoothstep(elapsed / duration)
        command = tuple(a + ratio * (b - a) for a, b in zip(start, target))
        node.publish_positions(command)
        rclpy.spin_once(node, timeout_sec=0.0)
        if elapsed >= duration:
            break
        time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = DirectMotionProbe()
    try:
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None:
            node.get_logger().error("MOTION_PROBE_NO_JOINT_STATE")
            return
        start = node.positions
        target = list(start)
        direction = 1.0 if start[0] <= 2.4 else -1.0
        target[0] = max(-2.7, min(2.7, start[0] + direction * 0.25))
        if not all(math.isfinite(value) for value in start):
            node.get_logger().error("MOTION_PROBE_INVALID_JOINT_STATE")
            return
        node.get_logger().info("MOTION_PROBE_OUTBOUND")
        _execute_segment(node, start, tuple(target))
        node.get_logger().info("MOTION_PROBE_RETURN")
        _execute_segment(node, tuple(target), start)
        for _ in range(10):
            node.publish_positions(start)
            time.sleep(1.0 / 120.0)
        node.get_logger().info("MOTION_PROBE_SUCCESS")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
