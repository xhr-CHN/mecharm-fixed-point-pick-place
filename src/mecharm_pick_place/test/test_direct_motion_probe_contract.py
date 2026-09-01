from pathlib import Path


SOURCE = Path(
    "src/mecharm_pick_place/mecharm_pick_place/direct_motion_probe.py"
).read_text(encoding="utf-8")


def test_probe_uses_smooth_named_joint_targets_without_moveit():
    assert '"/joint_states"' in SOURCE
    assert '"/mecharm/joint_target"' in SOURCE
    assert "_smoothstep" in SOURCE
    assert "MOTION_PROBE_SUCCESS" in SOURCE
    assert "moveit" not in SOURCE.lower()
