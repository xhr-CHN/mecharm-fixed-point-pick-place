from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_dockerfile_contains_clean_humble_moveit_runtime():
    dockerfile = (ROOT / "docker/Dockerfile").read_text(encoding="utf-8")
    for token in (
        "FROM osrf/ros:humble-desktop-full",
        "fastdds-tools",
        "ros-humble-moveit",
        "ros-humble-control-msgs",
        "python3-colcon-common-extensions",
    ):
        assert token in dockerfile
    assert "bobac" not in dockerfile.lower()


def test_compose_owns_one_host_networked_moveit_service():
    compose = yaml.safe_load((ROOT / "docker-compose.yml").read_text(encoding="utf-8"))
    assert compose["name"] == "mecharm-exp2"
    services = compose["services"]
    assert set(services) == {"moveit"}
    assert services["moveit"]["network_mode"] == "host"
    assert services["moveit"]["environment"]["ROS_DOMAIN_ID"] == "0"
    assert services["moveit"]["environment"]["ROS_LOCALHOST_ONLY"] == "1"
    assert "ROS_DISCOVERY_SERVER" not in services["moveit"]["environment"]
    assert "bobac" not in (ROOT / "docker-compose.yml").read_text(encoding="utf-8").lower()
