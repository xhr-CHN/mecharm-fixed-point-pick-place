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
TRANSPORT = os.environ.get("MECHARM_TRANSPORT", "tcp").strip().lower()
print(
    "Isaac transport environment: "
    f"transport={TRANSPORT}, "
    f"domain={os.environ.get('ROS_DOMAIN_ID', '<unset>')}, "
    f"rmw={os.environ.get('RMW_IMPLEMENTATION', '<unset>')}, "
    f"discovery={os.environ.get('ROS_DISCOVERY_SERVER', '<unset>')}",
    flush=True,
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
    self_collision_mode = os.environ.get(
        "MECHARM_SELF_COLLISION", "selective"
    ).strip().lower()
    if self_collision_mode not in ("off", "selective", "all"):
        raise RuntimeError(
            "MECHARM_SELF_COLLISION must be off, selective, or all"
        )
    import_config.set_self_collision(self_collision_mode != "off")
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

    if self_collision_mode == "selective":
        arm_links = ("base", "link1", "link2", "link3", "link4", "link5", "link6")
        gripper_links = (
            "gripper_base",
            "gripper_left1",
            "gripper_left2",
            "gripper_left3",
            "gripper_right1",
            "gripper_right2",
            "gripper_right3",
        )
        filtered_pairs = {
            ("base", "link1"),
            ("link1", "link2"),
            ("link2", "link3"),
            ("link3", "link4"),
            ("link4", "link5"),
            ("link5", "link6"),
            ("link6", "gripper_base"),
        }
        filtered_pairs.update(
            (arm_link, gripper_link)
            for arm_link in arm_links
            for gripper_link in gripper_links
        )
        filtered_pairs.update(
            (first, second)
            for index, first in enumerate(gripper_links)
            for second in gripper_links[index + 1 :]
            if {first, second} != {"gripper_left1", "gripper_right1"}
        )
        for first, second in sorted(filtered_pairs):
            first_path = f"{ROBOT_PRIM_PATH}/{first}"
            second_path = f"{ROBOT_PRIM_PATH}/{second}"
            first_prim = stage.GetPrimAtPath(first_path)
            second_prim = stage.GetPrimAtPath(second_path)
            if not first_prim.IsValid() or not second_prim.IsValid():
                raise RuntimeError(
                    f"cannot configure selective self-collision: {first_path}, {second_path}"
                )
            filtering = UsdPhysics.FilteredPairsAPI.Apply(first_prim)
            filtering.CreateFilteredPairsRel().AddTarget(Sdf.Path(second_path))
        print(
            "Configured selective robot self-collision; opposing final jaws remain active",
            flush=True,
        )
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

    # The official adaptive-gripper DAE files contain the linkage plates and
    # pivot holes, but no separate bolt/pin meshes.  PhysX revolute joints are
    # the actual constraints; these cylinders make the four exposed pivots
    # visually complete without adding collision geometry that could jam them.
    exposed_pins = (
        (
            "left_base_pin",
            f"{ROBOT_PRIM_PATH}/gripper_base",
            (-0.005, 0.027, -0.012),
            "gripper_base_to_gripper_left2",
        ),
        (
            "left_tip_pin",
            f"{ROBOT_PRIM_PATH}/gripper_left3",
            (-0.027, 0.016, -0.012),
            "gripper_left3_to_gripper_left1",
        ),
        (
            "right_base_pin",
            f"{ROBOT_PRIM_PATH}/gripper_base",
            (0.005, 0.027, -0.012),
            "gripper_base_to_gripper_right2",
        ),
        (
            "right_tip_pin",
            f"{ROBOT_PRIM_PATH}/gripper_right3",
            (0.027, 0.016, -0.012),
            "gripper_right3_to_gripper_right1",
        ),
    )
    for pin_name, parent_path, local_position, joint_name in exposed_pins:
        joint_path = f"{ROBOT_PRIM_PATH}/joints/{joint_name}"
        joint = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
        if not joint:
            raise RuntimeError(f"missing adaptive-gripper pivot joint: {joint_path}")
        if len(joint.GetBody0Rel().GetTargets()) != 1 or len(joint.GetBody1Rel().GetTargets()) != 1:
            raise RuntimeError(f"incomplete rigid-body connection at pivot: {joint_path}")
        parent = stage.GetPrimAtPath(parent_path)
        if not parent or not parent.IsValid():
            raise RuntimeError(f"missing adaptive-gripper pin parent: {parent_path}")
        pin_path = f"{parent_path}/visuals/physics_pins/{pin_name}"
        pin = UsdGeom.Cylinder.Define(stage, pin_path)
        pin.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        pin.CreateRadiusAttr().Set(0.0032)
        pin.CreateHeightAttr().Set(0.026)
        pin.CreateDisplayColorAttr().Set([(0.32, 0.34, 0.38)])
        UsdGeom.XformCommonAPI(pin).SetTranslate(local_position)

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
    if TRANSPORT == "ros2":
        enable_extension("isaacsim.ros2.bridge")
        for _ in range(30):
            simulation_app.update()
    elif TRANSPORT != "tcp":
        raise RuntimeError(f"unsupported MECHARM_TRANSPORT: {TRANSPORT}")

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

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
    robot = SingleArticulation(prim_path=ARTICULATION_ROOT_PATH, name="mecharm_270_pi")
    world.scene.add(robot)
    world.reset()
    robot.initialize()
    tcp_bridge = None
    if TRANSPORT == "tcp":
        from tcp_joint_bridge import IsaacTcpJointBridge

        tcp_bridge = IsaacTcpJointBridge(
            robot,
            bind_host=os.environ.get("MECHARM_TCP_BIND", "0.0.0.0"),
            port=int(os.environ.get("MECHARM_TCP_PORT", "8765")),
        )
    else:
        from ros2_graph import create_ros2_graph

        create_ros2_graph(ARTICULATION_ROOT_PATH)
        for _ in range(10):
            simulation_app.update()
    monitor = GraspMonitor(omni.usd.get_context().get_stage(), robot)
    max_steps = 30 if ARGS.smoke_test else None
    print(
        "Starting Isaac control loop: "
        f"transport={TRANSPORT}, "
        f"headless={ARGS.headless}, smoke_test={ARGS.smoke_test}, max_steps={max_steps}"
    )
    world.play()
    steps = 0
    try:
        while not simulation_app.is_exiting():
            if tcp_bridge is not None:
                tcp_bridge.apply_latest_command()
            world.step(render=not ARGS.headless)
            if tcp_bridge is not None:
                tcp_bridge.publish_state()
            else:
                from ros2_graph import tick_ros2_graph

                tick_ros2_graph()
            monitor.update()
            steps += 1
            if max_steps is not None and steps >= max_steps:
                print(f"Isaac control smoke test completed: {steps} simulation steps")
                break
    finally:
        if tcp_bridge is not None:
            tcp_bridge.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
