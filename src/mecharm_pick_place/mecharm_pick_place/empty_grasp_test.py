"""Attempt a grasp at an empty point, report the expected exception, and return home."""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy

from .cartesian_direct_pick_place import CartesianDirectPickPlace
from .direct_pick_place_demo import HOME, _move, _move_arm


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CartesianDirectPickPlace()
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None:
            raise RuntimeError("EMPTY_GRASP_NO_JOINT_STATE")

        command = node.positions
        # The initial scene places the object at y=+0.09; the calibrated
        # place point y=-0.08 is reachable and empty after a fresh scene load.
        empty_xyz = (0.18, -0.08, 0.025)
        pregrasp = float(node.get_parameter("pregrasp_clearance").value)
        pick_close = float(node.get_parameter("pick_close_clearance").value)
        open_position = float(node.get_parameter("gripper_open").value)
        closed_position = float(node.get_parameter("gripper_closed").value)
        node.get_logger().info("EMPTY_GRASP_TEST_START")
        command = node.move_gripper("EMPTY_TEST_OPEN", command, open_position)
        command = _move_arm(node, "HOME", command, HOME)

        home_tool_x = node.solver.forward(command[:6])[:3, 0]
        home_tool_x[2] = 0.0
        home_tool_x /= np.linalg.norm(home_tool_x)
        yaw = math.radians(float(node.get_parameter("grasp_yaw_offset_deg").value))
        c, s = math.cos(yaw), math.sin(yaw)
        grasp_tool_x = np.asarray(
            (c * home_tool_x[0] - s * home_tool_x[1],
             s * home_tool_x[0] + c * home_tool_x[1], 0.0),
            dtype=float,
        )

        command = _move(node, "EMPTY_PREGRASP", command, node.solve(
            "EMPTY_PREGRASP", empty_xyz, pregrasp, command, grasp_tool_x))
        command = _move(node, "EMPTY_GRASP", command, node.solve(
            "EMPTY_GRASP", empty_xyz, pick_close, command, grasp_tool_x))
        command = node.move_gripper("EMPTY_TEST_CLOSE", command, closed_position)
        node.get_logger().error(
            "EMPTY_GRASP_EXCEPTION code=NO_OBJECT message=gripper closed at empty test point"
        )
        command = node.move_gripper("EMPTY_TEST_OPEN_AFTER_EXCEPTION", command, open_position)
        command = _move(node, "EMPTY_RETREAT", command, node.solve(
            "EMPTY_RETREAT", empty_xyz, pregrasp, command, grasp_tool_x))
        _move_arm(node, "RETURN_HOME", command, HOME)
        node.get_logger().info("EMPTY_GRASP_RETURN_HOME")
    except RuntimeError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
