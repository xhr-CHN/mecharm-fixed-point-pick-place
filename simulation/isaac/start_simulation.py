"""Build and run the mechArm 270 Pi fixed-point pick-and-place scene."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import tempfile

from isaacsim import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--build-scene-only", action="store_true")
    parser.add_argument("--rebuild-scene", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    return parser.parse_args()


ARGS = parse_args()
PROJECT_ROOT = Path(ARGS.project_root).resolve()
SCENE_PATH = PROJECT_ROOT / "simulation/scenes/mecharm_pick_place.usd"

# Python 3.11 does not use PATH for dependent DLL lookup on Windows. Keep the
# Isaac ROS 2 runtime directory registered for the lifetime of the process.
ISAAC_SIM_ROOT = Path(os.environ.get("ISAAC_SIM_ROOT", r"E:\AIRobotic\isaac-sim"))
ROS2_RUNTIME = ISAAC_SIM_ROOT / "exts/isaacsim.ros2.bridge/humble/lib"
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
runtime_text = str(ROS2_RUNTIME)
if runtime_text.lower() not in os.environ.get("PATH", "").lower():
    os.environ["PATH"] = runtime_text + os.pathsep + os.environ.get("PATH", "")
ROS2_DLL_HANDLE = os.add_dll_directory(
    runtime_text
)

simulation_app = SimulationApp({"headless": ARGS.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage, open_stage, save_stage  # noqa: E402


URDF_PATH = (
    PROJECT_ROOT
    / "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
)
ROBOT_PRIM_PATH = "/World/mecharm_270_pi"
ARTICULATION_ROOT_PATH = f"{ROBOT_PRIM_PATH}/root_joint"


def build_scene() -> None:
    """Import the official URDF and create the repeatable experiment fixtures."""
    import numpy as np
    import omni.kit.commands
    import omni.usd
    from pxr import PhysxSchema, Sdf, UsdPhysics

    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
    from isaacsim.core.utils.prims import create_prim, move_prim

    enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()
    create_new_stage()
    create_prim("/World", "Xform")

    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("failed to create the Isaac Sim URDF import configuration")
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_default_prim = False
    import_config.create_physics_scene = True
    # The checked-in Isaac-specific URDF uses mutually compatible limits for
    # the complete adaptive-gripper mimic chain.
    import_config.parse_mimic = True
    import_config.set_self_collision(False)
    import_config.set_default_drive_strength(1e3)
    import_config.set_default_position_drive_damping(1e2)
    import_config.set_collision_from_visuals(True)
    # The 5.1 URDF importer cannot parse a source path containing Chinese
    # characters. Stage the same checked-in assets under a temporary ASCII path.
    source_description = URDF_PATH.parents[2]
    with tempfile.TemporaryDirectory(prefix="mecharm_exp2_") as temp_root:
        staged_description = Path(temp_root) / "mycobot_description"
        shutil.copytree(source_description, staged_description)
        staged_urdf = staged_description / URDF_PATH.relative_to(source_description)
        previous_cwd = Path.cwd()
        try:
            os.chdir(temp_root)
            status, imported_path = omni.kit.commands.execute(
                "URDFParseAndImportFile",
                urdf_path=str(staged_urdf),
                import_config=import_config,
            )
        finally:
            os.chdir(previous_cwd)
    if not status or not imported_path:
        raise RuntimeError(f"failed to import mechArm URDF: {URDF_PATH}")
    if imported_path != ROBOT_PRIM_PATH:
        move_prim(imported_path, ROBOT_PRIM_PATH)

    # Isaac Sim 5.1 may clear limits from imported mimic targets even when the
    # URDF contains finite limits. PhysX refuses to activate mimic joints
    # without finite angular limits, so restore them before the first reset.
    stage = omni.usd.get_context().get_stage()
    mimic_joints = {
        "gripper_base_to_gripper_left2": (-42.972, 8.594, 1.0),
        "gripper_left3_to_gripper_left1": (-8.594, 42.972, -1.0),
        "gripper_base_to_gripper_right3": (-8.594, 42.972, -1.0),
        "gripper_base_to_gripper_right2": (-8.594, 42.972, -1.0),
        "gripper_right3_to_gripper_right1": (-42.972, 8.594, 1.0),
    }
    reference_joint_path = f"{ROBOT_PRIM_PATH}/joints/gripper_controller"
    for joint_name, (lower, upper, urdf_multiplier) in mimic_joints.items():
        joint_path = f"{ROBOT_PRIM_PATH}/joints/{joint_name}"
        joint = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
        if not joint:
            raise RuntimeError(f"missing imported gripper joint: {joint_path}")
        joint.GetLowerLimitAttr().Set(lower)
        joint.GetUpperLimitAttr().Set(upper)
        mimic = PhysxSchema.PhysxMimicJointAPI.Apply(
            joint.GetPrim(), UsdPhysics.Tokens.rotZ
        )
        mimic.GetReferenceJointRel().SetTargets([Sdf.Path(reference_joint_path)])
        mimic.GetReferenceJointAxisAttr().Set(UsdPhysics.Tokens.rotZ)
        # PhysX defines gearing with the opposite sign to URDF multiplier.
        mimic.GetGearingAttr().Set(-urdf_multiplier)
        mimic.GetOffsetAttr().Set(0.0)

    world = World(stage_units_in_meters=1.0)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/work_table",
            name="work_table",
            position=np.array([0.14, 0.0, -0.025]),
            scale=np.array([0.60, 0.50, 0.05]),
            color=np.array([0.55, 0.40, 0.25]),
        )
    )
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/target_object",
            name="target_object",
            position=np.array([0.18, 0.08, 0.025]),
            scale=np.array([0.035, 0.035, 0.05]),
            color=np.array([0.85, 0.15, 0.10]),
            mass=0.03,
        )
    )
    world.reset()
    SCENE_PATH.parent.mkdir(parents=True, exist_ok=True)
    if not save_stage(str(SCENE_PATH)):
        raise RuntimeError(f"failed to save Isaac scene: {SCENE_PATH}")
    print(f"Saved mechArm experiment scene: {SCENE_PATH}")


def main() -> None:
    enable_extension("isaacsim.ros2.bridge")
    for _ in range(30):
        simulation_app.update()

    if ARGS.rebuild_scene or not SCENE_PATH.is_file():
        build_scene()
    if ARGS.build_scene_only:
        return

    if not open_stage(str(SCENE_PATH)):
        raise RuntimeError(f"failed to open Isaac scene: {SCENE_PATH}")
    for _ in range(10):
        simulation_app.update()

    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from grasp_monitor import GraspMonitor
    from ros2_graph import create_ros2_graph, tick_ros2_graph

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
    robot = SingleArticulation(prim_path=ARTICULATION_ROOT_PATH, name="mecharm_270_pi")
    world.scene.add(robot)
    world.reset()
    robot.initialize()
    create_ros2_graph(ARTICULATION_ROOT_PATH)
    # Let the OmniGraph nodes initialize before playback so the ROS 2
    # subscription is created, not just the publisher.
    for _ in range(10):
        simulation_app.update()
    monitor = GraspMonitor(omni.usd.get_context().get_stage(), robot)
    max_steps = 30 if ARGS.smoke_test else None
    print(
        "Starting Isaac ROS 2 graph loop: "
        f"headless={ARGS.headless}, smoke_test={ARGS.smoke_test}, max_steps={max_steps}"
    )
    world.play()
    steps = 0
    while not simulation_app.is_exiting():
        world.step(render=not ARGS.headless)
        tick_ros2_graph()
        monitor.update()
        steps += 1
        if max_steps is not None and steps >= max_steps:
            print(f"Isaac ROS 2 graph smoke test completed: {steps} simulation steps")
            break


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
