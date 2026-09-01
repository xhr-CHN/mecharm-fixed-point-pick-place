from pathlib import Path


SOURCE = Path("simulation/isaac/start_simulation.py").read_text(encoding="utf-8")


def test_selective_self_collision_keeps_only_opposing_final_jaws_active():
    assert '"MECHARM_SELF_COLLISION", "selective"' in SOURCE
    assert 'import_config.set_self_collision(self_collision_mode != "off")' in SOURCE
    assert "UsdPhysics.FilteredPairsAPI.Apply" in SOURCE
    assert '{"gripper_left1", "gripper_right1"}' in SOURCE
    assert "opposing final jaws remain active" in SOURCE
