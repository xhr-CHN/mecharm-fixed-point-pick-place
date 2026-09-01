from pathlib import Path
import yaml


def test_fixed_pick_place_parameters_match_isaac_scene():
    data = yaml.safe_load(
        Path("src/mecharm_moveit_demo/config/fixed_pick_place.yaml").read_text()
    )
    params = data["fixed_pick_place"]["ros__parameters"]
    assert params["pick_xyz"] == [0.18, 0.08, 0.025]
    assert params["place_xyz"] == [0.18, -0.08, 0.025]
    assert params["velocity_scaling"] == 0.20
    assert params["acceleration_scaling"] == 0.20
    assert params["cartesian_fraction_threshold"] == 0.95
    assert params["disable_collision_checking"] is True
