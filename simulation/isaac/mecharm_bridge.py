"""ROS 2 adapter for a verified mechArm articulation in Isaac Sim 5.1.

This adapter deliberately validates prims and DOF names at startup. It refuses
to move when the imported USD does not match the official URDF configuration.
"""

from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
import omni.usd
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


ARM_JOINTS = (
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
)
GRIPPER_JOINT = "gripper_controller"
GRIPPER_MIMIC_JOINTS = (
    ("gripper_controller", 1.0),
    ("gripper_base_to_gripper_left2", 1.0),
    ("gripper_left3_to_gripper_left1", -1.0),
    ("gripper_base_to_gripper_right3", -1.0),
    ("gripper_base_to_gripper_right2", -1.0),
    ("gripper_right3_to_gripper_right1", 1.0),
)
ROBOT_PRIM_PATH = "/World/mecharm_270_pi"
GRIPPER_BODY_PATH = f"{ROBOT_PRIM_PATH}/gripper_base"
OBJECT_PRIM_PATH = "/World/target_object"
GRASP_JOINT_PATH = "/World/mecharm_grasp_fixed_joint"


class MechArmBridge:
    def __init__(self, project_root: Path) -> None:
        self.project_root = Path(project_root)
        self.robot_prim_path = ROBOT_PRIM_PATH
        self.world = World(stage_units_in_meters=1.0)
        self.robot = Articulation(prim_paths_expr=self.robot_prim_path, name="mecharm_270_pi")
        self.world.scene.add(self.robot)
        self.world.reset()
        self.robot.initialize()

        actual_names = tuple(self.robot.dof_names)
        required_names = (*ARM_JOINTS, *(name for name, _ in GRIPPER_MIMIC_JOINTS))
        missing = [name for name in required_names if name not in actual_names]
        if missing:
            raise RuntimeError(
                f"imported articulation {self.robot_prim_path} is missing DOFs: {missing}; "
                f"available DOFs: {actual_names}"
            )

        description_path = self.project_root / "simulation/isaac/lula_robot_description.yaml"
        urdf_path = (
            self.project_root
            / "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
        )
        self.lula_solver = LulaKinematicsSolver(str(description_path), str(urdf_path))
        self.ik_solver = ArticulationKinematicsSolver(
            self.robot,
            self.lula_solver,
            "gripper_base",
        )

        enable_extension("isaacsim.ros2.bridge")
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64, String

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("isaac_mecharm_bridge")
        self.JointState = JointState
        self.String = String
        self.result_pub = self.node.create_publisher(String, "/mecharm/motion_result", 10)
        self.joint_pub = self.node.create_publisher(JointState, "/joint_states", 20)
        self.node.create_subscription(JointState, "/mecharm/joint_target", self._on_joint_target, 10)
        self.node.create_subscription(PoseStamped, "/mecharm/target_pose", self._on_pose_target, 10)
        self.node.create_subscription(Float64, "/mecharm/gripper_command", self._on_gripper, 10)
        self.last_publish = 0.0
        self.pending_indices = None
        self.pending_positions = None
        self.pending_kind = ""
        self.pending_started = 0.0
        self.joint_tolerance = 0.02
        self.command_timeout = 8.0
        self.attach_distance = 0.035

    def _publish_result(self, success: bool, error_code: str, message: str) -> None:
        payload = json.dumps(
            {"success": success, "error_code": error_code, "message": message},
            ensure_ascii=False,
            sort_keys=True,
        )
        self.result_pub.publish(self.String(data=payload))

    def _on_joint_target(self, msg) -> None:
        if self.pending_positions is not None:
            self._publish_result(False, "MOTION_FAILED", "another command is still active")
            return
        if len(msg.name) != len(msg.position):
            self._publish_result(False, "JOINT_LIMIT", "joint names and positions differ in length")
            return
        indices = []
        positions = []
        for name, position in zip(msg.name, msg.position):
            if name not in self.robot.dof_names:
                self._publish_result(False, "JOINT_LIMIT", f"unknown joint: {name}")
                return
            indices.append(self.robot.get_dof_index(name))
            positions.append(float(position))
        self._start_motion(indices, positions, "arm")

    def _on_pose_target(self, msg) -> None:
        if self.pending_positions is not None:
            self._publish_result(False, "MOTION_FAILED", "another command is still active")
            return
        position = np.asarray(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        quaternion_wxyz = np.asarray(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
            dtype=float,
        )
        action, success = self.ik_solver.compute_inverse_kinematics(
            target_position=position,
            target_orientation=quaternion_wxyz,
        )
        if not success or action.joint_positions is None:
            self._publish_result(False, "NO_IK", "Lula could not solve the requested pose")
            return
        indices = action.joint_indices
        if indices is None:
            indices = np.arange(len(action.joint_positions), dtype=np.int32)
        self._start_motion(indices, action.joint_positions, "arm")

    def _on_gripper(self, msg) -> None:
        if self.pending_positions is not None:
            self._publish_result(False, "GRIPPER_FAILED", "another command is still active")
            return
        opening = max(0.0, min(1.0, float(msg.data)))
        lower, upper = -0.74, 0.15
        target = lower + opening * (upper - lower)
        indices = [self.robot.get_dof_index(name) for name, _ in GRIPPER_MIMIC_JOINTS]
        positions = [target * multiplier for _, multiplier in GRIPPER_MIMIC_JOINTS]
        if opening >= 0.9:
            self._detach_object()
            kind = "gripper_open"
        else:
            kind = "gripper_close"
        self._start_motion(indices, positions, kind)

    def _start_motion(self, indices, positions, kind: str) -> None:
        self.pending_indices = np.asarray(indices, dtype=np.int32)
        self.pending_positions = np.asarray(positions, dtype=float)
        self.pending_kind = kind
        self.pending_started = time.monotonic()
        self.robot.get_articulation_controller().apply_action(
            ArticulationAction(
                joint_positions=self.pending_positions,
                joint_indices=self.pending_indices,
            )
        )

    def _check_pending_motion(self) -> None:
        if self.pending_positions is None:
            return
        current = np.asarray(self.robot.get_joint_positions(), dtype=float)[self.pending_indices]
        if np.max(np.abs(current - self.pending_positions)) <= self.joint_tolerance:
            kind = self.pending_kind
            self._clear_pending()
            if kind == "gripper_close" and not self._attach_object():
                self._publish_result(
                    False,
                    "OBJECT_OUT_OF_RANGE",
                    "target object is not within the allowed grasp distance",
                )
            else:
                self._publish_result(True, "NONE", f"{kind} target reached")
        elif time.monotonic() - self.pending_started > self.command_timeout:
            kind = self.pending_kind
            self._clear_pending()
            code = "GRIPPER_FAILED" if kind.startswith("gripper") else "TIMEOUT"
            self._publish_result(False, code, f"{kind} did not reach its target")

    def _clear_pending(self) -> None:
        self.pending_indices = None
        self.pending_positions = None
        self.pending_kind = ""
        self.pending_started = 0.0

    def _world_translation(self, prim_path: str) -> np.ndarray | None:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        value = matrix.ExtractTranslation()
        return np.asarray([value[0], value[1], value[2]], dtype=float)

    def _attach_object(self) -> bool:
        stage = omni.usd.get_context().get_stage()
        gripper_position = self._world_translation(GRIPPER_BODY_PATH)
        object_position = self._world_translation(OBJECT_PRIM_PATH)
        if gripper_position is None or object_position is None:
            return False
        if np.linalg.norm(gripper_position - object_position) > self.attach_distance:
            return False
        if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
            stage.RemovePrim(GRASP_JOINT_PATH)
        joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(GRIPPER_BODY_PATH)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(OBJECT_PRIM_PATH)])
        return True

    def _detach_object(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
            stage.RemovePrim(GRASP_JOINT_PATH)

    def _publish_joint_state(self) -> None:
        msg = self.JointState()
        now = self.node.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.robot.dof_names)
        msg.position = [float(value) for value in self.robot.get_joint_positions()]
        velocities = self.robot.get_joint_velocities()
        if velocities is not None:
            msg.velocity = [float(value) for value in velocities]
        self.joint_pub.publish(msg)

    def run(self, simulation_app, render: bool = True, max_steps: int | None = None) -> None:
        self.world.play()
        steps = 0
        while not simulation_app.is_exiting():
            self.world.step(render=render)
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            self._check_pending_motion()
            now = time.monotonic()
            if now - self.last_publish >= 0.05:
                self._publish_joint_state()
                self.last_publish = now
            steps += 1
            if max_steps is not None and steps >= max_steps:
                break
        if max_steps is not None:
            print(f"Isaac bridge smoke test completed: {steps} simulation steps")
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
