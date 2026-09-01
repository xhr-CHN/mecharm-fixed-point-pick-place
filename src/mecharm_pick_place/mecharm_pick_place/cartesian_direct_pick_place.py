"""URDF numerical IK followed by smooth direct joint execution."""

from __future__ import annotations

import time

import rclpy

from .direct_motion_probe import DirectMotionProbe, _execute_segment
from .direct_pick_place_demo import HOME, MIN_DURATION_S, _move, _move_arm
from .numeric_ik import UrdfNumericIK


class CartesianDirectPickPlace(DirectMotionProbe):
    def __init__(self):
        super().__init__(node_name="cartesian_direct_pick_place")
        self.declare_parameter("pick_xyz", [0.18, 0.08, 0.025])
        self.declare_parameter("place_xyz", [0.18, -0.08, 0.025])
        self.declare_parameter("pregrasp_clearance", 0.08)
        self.declare_parameter("grasp_clearance", 0.0)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("vertical_axis_dot_min", 0.98)
        self.declare_parameter("gripper_open", 0.15)
        self.declare_parameter("gripper_closed", -0.75)
        self.declare_parameter("gripper_speed_rad_s", 0.12)
        self.declare_parameter("gripper_wait_before_sec", 1.0)
        self.declare_parameter("gripper_wait_after_sec", 1.5)
        self.solver = UrdfNumericIK(str(self.get_parameter("urdf_path").value))

    def move_gripper(self, label, command, target_position):
        speed = float(self.get_parameter("gripper_speed_rad_s").value)
        wait_before = float(self.get_parameter("gripper_wait_before_sec").value)
        wait_after = float(self.get_parameter("gripper_wait_after_sec").value)
        target = command[:6] + (float(target_position),)
        duration = max(MIN_DURATION_S, abs(target[6] - command[6]) / speed)
        self.get_logger().info(f"GRIPPER_WAIT_BEFORE {label} {wait_before:.2f}s")
        time.sleep(wait_before)
        self.get_logger().info(
            f"DIRECT_STAGE {label} duration={duration:.2f}s speed={speed:.3f}rad/s"
        )
        _execute_segment(self, command, target, duration=duration)
        self.get_logger().info(f"GRIPPER_WAIT_AFTER {label} {wait_after:.2f}s")
        time.sleep(wait_after)
        return target

    def solve(self, label, object_xyz, clearance, seed):
        xyz = [float(value) for value in object_xyz]
        target = (xyz[0], xyz[1], xyz[2] + float(clearance))
        self.get_logger().info(
            f"NUMERIC_IK_REQUEST {label} xyz=({target[0]:.3f},"
            f"{target[1]:.3f},{target[2]:.3f})"
        )
        positions, position_error, axis_dot = self.solver.solve(
            target,
            seed[:6],
            position_tolerance=float(self.get_parameter("position_tolerance").value),
            axis_dot_min=float(self.get_parameter("vertical_axis_dot_min").value),
        )
        self.get_logger().info(
            f"NUMERIC_IK_SUCCESS {label} position_error={position_error:.4f} "
            f"axis_dot={axis_dot:.4f}"
        )
        return positions + (seed[6],)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CartesianDirectPickPlace()
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and node.positions is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.positions is None:
            raise RuntimeError("CARTESIAN_DIRECT_NO_JOINT_STATE")
        pick = node.get_parameter("pick_xyz").value
        place = node.get_parameter("place_xyz").value
        pregrasp = float(node.get_parameter("pregrasp_clearance").value)
        grasp = float(node.get_parameter("grasp_clearance").value)
        open_position = float(node.get_parameter("gripper_open").value)
        closed_position = float(node.get_parameter("gripper_closed").value)
        command = node.positions
        node.get_logger().info("CARTESIAN_DIRECT_PICK_PLACE_START")
        command = node.move_gripper("INITIAL_GRIPPER_CLOSE", command, closed_position)
        command = _move_arm(node, "HOME", command, HOME)
        command = _move(node, "PICK_PREGRASP", command, node.solve("PICK_PREGRASP", pick, pregrasp, command))
        command = node.move_gripper("GRIPPER_OPEN_ABOVE_OBJECT", command, open_position)
        command = _move(node, "PICK_GRASP", command, node.solve("PICK_GRASP", pick, grasp, command))
        command = node.move_gripper("GRIPPER_CLOSE_ON_OBJECT", command, closed_position)
        command = _move(node, "LIFT", command, node.solve("LIFT", pick, pregrasp, command))
        command = _move(node, "PLACE_PREGRASP", command, node.solve("PLACE_PREGRASP", place, pregrasp, command))
        command = _move(node, "PLACE_GRASP", command, node.solve("PLACE_GRASP", place, grasp, command))
        command = node.move_gripper("GRIPPER_OPEN_TO_RELEASE", command, open_position)
        command = _move(node, "RETREAT", command, node.solve("RETREAT", place, pregrasp, command))
        _move_arm(node, "RETURN_HOME", command, HOME)
        node.get_logger().info("CARTESIAN_DIRECT_PICK_PLACE_SUCCESS")
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
