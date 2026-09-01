from pathlib import Path
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
    assert (chain.attrib["base_link"], chain.attrib["tip_link"]) == ("base", "gripper_base")
    limits = yaml.safe_load((CONFIG / "joint_limits.yaml").read_text())["joint_limits"]
    assert all(limits[name]["max_velocity"] > 0.0 for name in ARM)
    controllers = yaml.safe_load((CONFIG / "moveit_controllers.yaml").read_text())
    assert controllers["moveit_simple_controller_manager"]["mecharm_controller"]["joints"] == ARM
