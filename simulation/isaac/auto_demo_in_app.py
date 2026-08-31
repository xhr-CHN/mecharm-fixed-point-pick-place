"""Run the calibrated pick-and-place demo inside an open Isaac Sim window."""

import asyncio
import time
import traceback

import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


ROBOT_PATH = "/World/mecharm_270_pi"
OBJECT_PATH = "/World/target_object"
GRIPPER_BODY_PATH = f"{ROBOT_PATH}/gripper_base"
GRASP_JOINT_PATH = "/World/mecharm_grasp_fixed_joint"

ARM_JOINTS = (
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
)
GRIPPER_JOINT = "gripper_controller"
PHYSICS_HZ = 120.0
RENDER_HZ = 60.0
MAX_ARM_SPEED_DEG_S = 12.0
MAX_GRIPPER_SPEED_RAD_S = 0.25
MIN_MOTION_DURATION_S = 1.0
SETTLE_DURATION_S = 0.35
ENABLE_OBJECT_ATTACHMENT = True
ATTACH_DISTANCE_M = 0.12

HOME = (0.0, 0.0, 0.0, -47.0, 0.0, 0.0)
PICK_SAFE = (23.093, -0.417, 18.790, -47.647, -2.617, 0.0)
PICK_GRASP = (23.363, 14.921, 20.671, -47.479, -1.886, 0.0)
PLACE_SAFE = (-19.315, 0.478, 13.209, -47.445, 12.505, 0.0)
PLACE_GRASP = (-19.656, 15.759, 15.709, -47.310, 11.583, 0.0)


async def _wait_frames(count: int) -> None:
    app = omni.kit.app.get_app()
    for _ in range(count):
        await app.next_update_async()


async def _run_demo() -> None:
    stage = omni.usd.get_context().get_stage()
    for required_path in (ROBOT_PATH, OBJECT_PATH):
        if not stage.GetPrimAtPath(required_path).IsValid():
            raise RuntimeError(f"required scene prim is missing: {required_path}")

    # Opening a USD invalidates any World wrapper created for the previous
    # stage. Recreate it around the currently open scene before controlling it.
    World.clear_instance()
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / PHYSICS_HZ,
        rendering_dt=1.0 / RENDER_HZ,
    )
    robot = SingleArticulation(prim_path=ROBOT_PATH, name="mecharm_auto_demo")
    world.scene.add(robot)
    await world.initialize_simulation_context_async()
    await world.reset_async()
    await world.play_async()
    robot.initialize()

    arm_indices = np.asarray([robot.get_dof_index(name) for name in ARM_JOINTS], dtype=np.int32)
    gripper_indices = np.asarray([robot.get_dof_index(GRIPPER_JOINT)], dtype=np.int32)
    controller = robot.get_articulation_controller()

    kps, kds = controller.get_gains()
    kps[arm_indices] = 1000.0
    kds[arm_indices] = 100.0
    controller.set_gains(kps=kps, kds=kds)

    async def wait_simulation_time(duration_s):
        if not world.is_playing():
            await world.play_async()
        started_at = world.current_time
        deadline = time.monotonic() + max(5.0, duration_s * 4.0)
        while world.current_time - started_at < duration_s:
            if time.monotonic() > deadline:
                raise RuntimeError("simulation time stopped advancing")
            await _wait_frames(1)

    async def move_joints(label, indices, target, max_speed):
        current_all = robot.get_joint_positions()
        if current_all is None:
            raise RuntimeError("joint positions are unavailable after initialization")
        start = np.asarray(current_all, dtype=float)[indices]
        target = np.asarray(target, dtype=float)
        largest_move = float(np.max(np.abs(target - start)))
        duration_s = max(MIN_MOTION_DURATION_S, largest_move / max_speed)
        elapsed_s = 0.0
        motion_done = False
        callback_name = "mecharm_fixed_rate_trajectory"

        def update_trajectory(step_size):
            nonlocal elapsed_s, motion_done
            elapsed_s += float(step_size)
            phase = min(elapsed_s / duration_s, 1.0)
            blend = phase**3 * (10.0 + phase * (-15.0 + 6.0 * phase))
            positions = start + (target - start) * blend
            controller.apply_action(
                ArticulationAction(joint_positions=positions, joint_indices=indices)
            )
            if phase >= 1.0:
                motion_done = True

        print(
            f"[mechArm demo] moving to {label}: "
            f"{duration_s:.2f}s at {PHYSICS_HZ:.0f} Hz"
        )
        if not world.is_playing():
            await world.play_async()
        if world.physics_callback_exists(callback_name):
            world.remove_physics_callback(callback_name)
        world.add_physics_callback(callback_name, update_trajectory)
        deadline = time.monotonic() + max(8.0, duration_s * 4.0)
        try:
            while not motion_done:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        f"physics callback did not complete trajectory: {label}"
                    )
                await _wait_frames(1)
        finally:
            if world.physics_callback_exists(callback_name):
                world.remove_physics_callback(callback_name)

        controller.apply_action(
            ArticulationAction(joint_positions=target, joint_indices=indices)
        )
        await wait_simulation_time(SETTLE_DURATION_S)

    async def move_arm(label, target_deg):
        await move_joints(
            label,
            arm_indices,
            np.deg2rad(target_deg),
            np.deg2rad(MAX_ARM_SPEED_DEG_S),
        )

    async def move_gripper(label, target):
        await move_joints(
            label,
            gripper_indices,
            [target],
            MAX_GRIPPER_SPEED_RAD_S,
        )

    def attach_object():
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
            stage.RemovePrim(GRASP_JOINT_PATH)
        gripper_prim = stage.GetPrimAtPath(GRIPPER_BODY_PATH)
        object_prim = stage.GetPrimAtPath(OBJECT_PATH)
        time_code = Usd.TimeCode.Default()
        gripper_world = UsdGeom.Xformable(gripper_prim).ComputeLocalToWorldTransform(time_code)
        object_world = UsdGeom.Xformable(object_prim).ComputeLocalToWorldTransform(time_code)
        distance = np.linalg.norm(
            np.asarray(object_world.ExtractTranslation())
            - np.asarray(gripper_world.ExtractTranslation())
        )
        if distance > ATTACH_DISTANCE_M:
            raise RuntimeError(
                f"object is {distance:.3f}m from the gripper base; "
                f"attachment limit is {ATTACH_DISTANCE_M:.3f}m"
            )
        gripper_to_object = object_world * gripper_world.GetInverse()
        joint = UsdPhysics.FixedJoint.Define(stage, GRASP_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(GRIPPER_BODY_PATH)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(OBJECT_PATH)])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(gripper_to_object.ExtractTranslation()))
        joint.CreateLocalRot0Attr().Set(
            Gf.Quatf(gripper_to_object.ExtractRotation().GetQuat())
        )
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    def detach_object():
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(GRASP_JOINT_PATH).IsValid():
            stage.RemovePrim(GRASP_JOINT_PATH)

    print("[mechArm demo] starting automatic pick-and-place")
    detach_object()
    await move_gripper("gripper open", 0.15)
    await move_arm("home", HOME)
    await move_arm("pick safe", PICK_SAFE)
    await move_arm("pick grasp", PICK_GRASP)
    await move_gripper("gripper closed", -0.75)
    if ENABLE_OBJECT_ATTACHMENT:
        attach_object()
    await wait_simulation_time(SETTLE_DURATION_S)
    await move_arm("pick safe", PICK_SAFE)
    await move_arm("central transit", HOME)
    await move_arm("place safe", PLACE_SAFE)
    await move_arm("place grasp", PLACE_GRASP)
    detach_object()
    await move_gripper("gripper open", 0.15)
    await move_arm("place safe", PLACE_SAFE)
    await move_arm("home", HOME)
    if ENABLE_OBJECT_ATTACHMENT:
        print("[mechArm demo] automatic pick-and-place completed")
    else:
        print("[mechArm demo] stable-motion validation completed; object attachment disabled")


def _report_demo_result(task) -> None:
    try:
        task.result()
    except asyncio.CancelledError:
        print("[mechArm demo] cancelled")
    except Exception:
        print("[mechArm demo] failed")
        traceback.print_exc()


async def _restart_demo(previous_task) -> None:
    if previous_task is not None and not previous_task.done():
        print("[mechArm demo] cancelling previous task")
        previous_task.cancel()
        try:
            await previous_task
        except asyncio.CancelledError:
            pass
    await _run_demo()


previous_demo_task = globals().get("MECHARM_DEMO_TASK")
MECHARM_DEMO_TASK = asyncio.ensure_future(_restart_demo(previous_demo_task))
MECHARM_DEMO_TASK.add_done_callback(_report_demo_result)
print("[mechArm demo] task scheduled")
