from pathlib import Path
import xml.etree.ElementTree as ET


SOURCE = Path("simulation/isaac/start_simulation.py").read_text(encoding="utf-8")
URDF = Path(
    "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/"
    "mecharm_270_pi_adaptive_gripper.urdf"
)


def test_four_exposed_gripper_pins_have_visuals_and_physical_joint_validation():
    for pin in (
        "left_base_pin",
        "left_tip_pin",
        "right_base_pin",
        "right_tip_pin",
    ):
        assert pin in SOURCE
    for joint in (
        "gripper_base_to_gripper_left2",
        "gripper_left_loop_joint",
        "gripper_base_to_gripper_right2",
        "gripper_right_loop_joint",
    ):
        assert joint in SOURCE
    assert "UsdGeom.Cylinder.Define" in SOURCE
    assert "GetBody0Rel().GetTargets()" in SOURCE
    assert "GetBody1Rel().GetTargets()" in SOURCE
    assert "UsdPhysics.CollisionAPI.Apply" not in SOURCE
    assert 'pin_path = f"{parent_path}/physics_pins/{pin_name}"' in SOURCE
    assert '/visuals/physics_pins/' not in SOURCE
    assert 'f"{ROBOT_PRIM_PATH}/gripper_left1"' in SOURCE
    assert "(0.006037, 0.021311, -0.012)" in SOURCE
    assert 'f"{ROBOT_PRIM_PATH}/gripper_right1"' in SOURCE
    assert "(-0.006037, 0.021311, -0.012)" in SOURCE


def test_outer_jaws_are_passive_and_closed_by_two_loop_joints():
    assert "RemoveAPI(UsdPhysics.DriveAPI" in SOURCE
    assert '"gripper_left_loop_joint"' in SOURCE
    assert '"gripper_right_loop_joint"' in SOURCE
    assert "(-0.027963, 0.015311, 0.0)" in SOURCE
    assert "(0.006037, 0.021311, 0.0)" in SOURCE
    assert "CreateExcludeFromArticulationAttr().Set(True)" in SOURCE
    assert "CreateSolverPositionIterationCountAttr().Set(64)" in SOURCE
    assert "CreateSolverVelocityIterationCountAttr().Set(64)" in SOURCE
    assert "Created left/right adaptive-gripper loop pin constraints" in SOURCE
    root = ET.parse(URDF).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert joints["gripper_left3_to_gripper_left1"].find("mimic") is None
    assert joints["gripper_right3_to_gripper_right1"].find("mimic") is None
