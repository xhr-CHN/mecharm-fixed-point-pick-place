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

simulation_app = SimulationApp({"headless": ARGS.headless})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage, open_stage, save_stage  # noqa: E402


URDF_PATH = (
    PROJECT_ROOT
    / "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
)
ROBOT_PRIM_PATH = "/World/mecharm_270_pi"


def build_scene() -> None:
    """Import the official URDF and create the repeatable experiment fixtures."""
    import numpy as np
    import omni.kit.commands

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
    # Isaac Sim 5.1 rejects several valid mimic limits in this vendor URDF.
    # Import all gripper joints as DOFs; the bridge applies the mimic mapping.
    import_config.parse_mimic = False
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
    os.environ.setdefault("ROS_DISTRO", "humble")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    if ARGS.rebuild_scene or not SCENE_PATH.is_file():
        build_scene()
    if ARGS.build_scene_only:
        return

    if not open_stage(str(SCENE_PATH)):
        raise RuntimeError(f"failed to open Isaac scene: {SCENE_PATH}")

    from mecharm_bridge import MechArmBridge

    bridge = MechArmBridge(PROJECT_ROOT)
    bridge.run(
        simulation_app,
        render=not ARGS.headless,
        max_steps=30 if ARGS.smoke_test else None,
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
