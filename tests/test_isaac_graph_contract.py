from pathlib import Path


GRAPH = Path("simulation/isaac/ros2_graph.py").read_text(encoding="utf-8")


def test_graph_uses_compiled_bridge_nodes_and_expected_topics():
    for token in (
        "ROS2PublishJointState",
        "ROS2SubscribeJointState",
        "IsaacArticulationController",
        '"/joint_states"',
        '"/mecharm/joint_target"',
    ):
        assert token in GRAPH
    assert "import rclpy" not in GRAPH
