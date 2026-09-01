from pathlib import Path
from itertools import combinations
import xml.etree.ElementTree as ET
import yaml


ROOT = Path(__file__).parents[3]
CONFIG = ROOT / "src/mecharm_moveit_config/config"
ARM = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
       "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]


def test_arm_and_controller_contract():
    root = ET.parse(CONFIG / "mecharm_270_pi.srdf").getroot()
    group = next(item for item in root.findall("group") if item.attrib["name"] == "arm")
    chain = group.find("chain")
    assert (chain.attrib["base_link"], chain.attrib["tip_link"]) == ("base", "gripper_tcp")
    limits = yaml.safe_load((CONFIG / "joint_limits.yaml").read_text())["joint_limits"]
    assert all(limits[name]["max_velocity"] > 0.0 for name in ARM)
    controllers = yaml.safe_load((CONFIG / "moveit_controllers.yaml").read_text())
    assert controllers["moveit_simple_controller_manager"]["mecharm_controller"]["joints"] == ARM


def test_rviz_uses_world_frame_and_moveit_display():
    rviz = (CONFIG / "moveit.rviz").read_text(encoding="utf-8")
    assert "Fixed Frame: world" in rviz
    assert "Class: rviz_default_plugins/RobotModel" in rviz
    assert "Class: moveit_rviz_plugin/MotionPlanning" in rviz
    assert "Planning Scene Topic: /monitored_planning_scene" in rviz


def test_all_adaptive_gripper_internal_collisions_are_disabled():
    root = ET.parse(CONFIG / "mecharm_270_pi.srdf").getroot()
    disabled = {
        frozenset((item.attrib["link1"], item.attrib["link2"]))
        for item in root.findall("disable_collisions")
    }
    gripper_links = (
        "gripper_base",
        "gripper_left1",
        "gripper_left2",
        "gripper_left3",
        "gripper_right1",
        "gripper_right2",
        "gripper_right3",
    )
    assert all(frozenset(pair) in disabled for pair in combinations(gripper_links, 2))
