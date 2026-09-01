from pathlib import Path


ROOT = Path(__file__).parents[1]
ISAAC = (ROOT / "simulation/isaac/tcp_joint_bridge.py").read_text(encoding="utf-8")
ROS = (
    ROOT / "src/mecharm_pick_place/mecharm_pick_place/tcp_bridge_node.py"
).read_text(encoding="utf-8")


def test_tcp_bridge_preserves_existing_ros_topic_contract():
    assert '"/joint_states"' in ROS
    assert '"/mecharm/joint_target"' in ROS
    assert '"type": "command"' in ROS
    assert '"type": "state"' in ISAAC
    for joint in (
        "joint1_to_base",
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "gripper_controller",
    ):
        assert joint in ISAAC


def test_grasp_monitor_does_not_attach_from_pregrasp_height():
    monitor = (ROOT / "simulation/isaac/grasp_monitor.py").read_text(encoding="utf-8")
    assert "attach_distance=0.06" in monitor
