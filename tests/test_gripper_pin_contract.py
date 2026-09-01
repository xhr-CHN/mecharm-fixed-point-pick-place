from pathlib import Path


SOURCE = Path("simulation/isaac/start_simulation.py").read_text(encoding="utf-8")


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
        "gripper_left3_to_gripper_left1",
        "gripper_base_to_gripper_right2",
        "gripper_right3_to_gripper_right1",
    ):
        assert joint in SOURCE
    assert "UsdGeom.Cylinder.Define" in SOURCE
    assert "GetBody0Rel().GetTargets()" in SOURCE
    assert "GetBody1Rel().GetTargets()" in SOURCE
    assert "UsdPhysics.CollisionAPI.Apply" not in SOURCE
