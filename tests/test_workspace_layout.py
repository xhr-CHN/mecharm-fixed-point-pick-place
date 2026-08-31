from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_required_workspace_files_exist():
    required = [
        ROOT / "config/simulation.yaml",
        ROOT / "src/mecharm_pick_place/package.xml",
        ROOT / "src/mecharm_pick_place/launch/simulation.launch.py",
        ROOT / "simulation/urdf/mycobot_description/LICENSE",
        ROOT / "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf",
    ]
    missing = [str(path.relative_to(ROOT)) for path in required if not path.is_file()]
    assert not missing, f"missing workspace files: {missing}"


def test_vendor_urdf_is_mecharm_pi_with_adaptive_gripper():
    urdf = ROOT / "simulation/urdf/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf"
    robot = ET.parse(urdf).getroot()
    joints = {joint.attrib["name"] for joint in robot.findall("joint")}
    assert "joint1_to_base" in joints
    assert "joint6_to_joint5" in joints
    assert "gripper_controller" in joints
