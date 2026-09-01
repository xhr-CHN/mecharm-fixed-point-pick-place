# MoveIt 2–Isaac Trajectory Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hard-coded Isaac joint-waypoint demo with a collision-aware MoveIt 2 pick-and-place flow that executes complete `FollowJointTrajectory` goals smoothly in Isaac Sim 5.1.

**Architecture:** ROS 2 Humble in WSL runs MoveIt, the task node, and a standard trajectory action adapter. Isaac Sim runs on Windows; a programmatically-created ROS 2 OmniGraph exchanges named joint targets and measured joint states, while an Isaac physics callback owns only physical object attachment.

**Tech Stack:** Windows 11, Isaac Sim 5.1.0, WSL 2 Ubuntu 22.04, ROS 2 Humble, MoveIt 2, OMPL, KDL, `control_msgs`, `rclpy`, C++17, Python 3.10, pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-09-01-moveit2-isaac-trajectory-design.md`

## Global Constraints

- Isaac Sim 5.1.0 remains on Windows at `E:\AIRobotic\isaac-sim`; ROS 2 Humble and MoveIt 2 run in WSL `Ubuntu-22.04`.
- Scope ends with working simulation; do not add Docker, Jetson, or a real-arm driver.
- Arm order is exactly `joint1_to_base`, `joint2_to_joint1`, `joint3_to_joint2`, `joint4_to_joint3`, `joint5_to_joint4`, `joint6_to_joint5`.
- The controller action is `/mecharm_controller/follow_joint_trajectory`; command and feedback topics are `/mecharm/joint_target` and `/joint_states`.
- Preserve every MoveIt trajectory point and its `time_from_start`; default velocity and acceleration scaling are both `0.20`.
- The adaptive gripper master is `gripper_controller`; mimic follower joints never enter the MoveIt arm group.
- The cube starts at `(0.18, 0.08, 0.025)` m; nominal place center is `(0.18, -0.08, 0.025)` m; table center/size are `(0.14, 0.0, -0.025)` and `(0.60, 0.50, 0.05)` m.
- Do not commit generated `build/`, `install/`, `log/`, `__pycache__/`, or `simulation/scenes/*.usd` files.

## File Structure

- `simulation/urdf/mycobot_description/package.xml` and `CMakeLists.txt`: expose the existing URDF and meshes as one ROS 2 description package without duplicating assets.
- `src/mecharm_moveit_config/config/*`: SRDF, kinematics, joint limits, OMPL, and controller mapping.
- `src/mecharm_moveit_config/launch/move_group.launch.py`: publish the robot description and start `move_group`; RViz remains optional.
- `src/mecharm_pick_place/mecharm_pick_place/trajectory.py`: ROS-independent validation, joint reordering, and interpolation.
- `src/mecharm_pick_place/mecharm_pick_place/trajectory_controller_node.py`: `FollowJointTrajectory` action server and gripper command merger.
- `simulation/isaac/ros2_graph.py`: create and validate compiled Isaac ROS bridge OmniGraph nodes.
- `simulation/isaac/grasp_monitor.py`: attach/detach the cube from measured physics state only.
- `src/mecharm_moveit_demo/src/fixed_pick_place_moveit.cpp`: collision-scene setup and MoveIt pick/place sequence.
- `scripts/start_moveit_sim.ps1`: deterministic two-process startup instructions and preflight checks.

---

### Task 1: Install and verify the WSL MoveIt toolchain

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: WSL distribution `Ubuntu-22.04` with `/opt/ros/humble/setup.bash`.
- Produces: installed packages `moveit_ros_move_group`, `moveit_configs_utils`, `control_msgs`, `trajectory_msgs`, and `xacro`.

- [ ] **Step 1: Record the current package preflight**

Run in PowerShell:

```powershell
wsl.exe -d Ubuntu-22.04 -u xhr -- bash -lc "source /opt/ros/humble/setup.bash; for p in moveit_ros_move_group moveit_configs_utils control_msgs trajectory_msgs xacro; do ros2 pkg prefix `$p >/dev/null 2>&1 && echo PASS:`$p || echo MISSING:`$p; done"
```

Expected before installation: at least `MISSING:moveit_ros_move_group` and `MISSING:moveit_configs_utils`.

- [ ] **Step 2: Install only the simulation dependencies**

Run in WSL:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-moveit ros-humble-moveit-configs-utils ros-humble-control-msgs ros-humble-trajectory-msgs ros-humble-xacro
```

- [ ] **Step 3: Verify package discovery**

Run:

```bash
source /opt/ros/humble/setup.bash
for p in moveit_ros_move_group moveit_configs_utils control_msgs trajectory_msgs xacro; do ros2 pkg prefix "$p"; done
```

Expected: five package prefixes under `/opt/ros/humble` and exit code 0.

- [ ] **Step 4: Add exact prerequisite commands to README**

Under a `WSL prerequisites` heading in `README.md`, include the install command from Step 2, the required distribution name, and state that Isaac must run on Windows with `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "docs: add MoveIt 2 simulation prerequisites"
```

### Task 2: Package the existing robot description and add MoveIt configuration

**Files:**
- Create: `simulation/urdf/mycobot_description/package.xml`
- Create: `simulation/urdf/mycobot_description/CMakeLists.txt`
- Create: `src/mecharm_moveit_config/package.xml`
- Create: `src/mecharm_moveit_config/CMakeLists.txt`
- Create: `src/mecharm_moveit_config/config/mecharm_270_pi.srdf`
- Create: `src/mecharm_moveit_config/config/kinematics.yaml`
- Create: `src/mecharm_moveit_config/config/joint_limits.yaml`
- Create: `src/mecharm_moveit_config/config/ompl_planning.yaml`
- Create: `src/mecharm_moveit_config/config/moveit_controllers.yaml`
- Create: `src/mecharm_moveit_config/launch/move_group.launch.py`
- Test: `src/mecharm_moveit_config/test/test_moveit_config.py`

**Interfaces:**
- Consumes: `mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf` and its `package://mycobot_description/...` mesh paths.
- Produces: MoveIt group `arm`, end link `gripper_base`, controller `mecharm_controller`, and launch argument `use_rviz: bool`.

- [ ] **Step 1: Write a failing configuration test**

```python
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).parents[3]
CONFIG = ROOT / "src/mecharm_moveit_config/config"
ARM = ["joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
       "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5"]

def test_srdf_arm_contains_only_six_arm_joints():
    root = ET.parse(CONFIG / "mecharm_270_pi.srdf").getroot()
    group = next(item for item in root.findall("group") if item.attrib["name"] == "arm")
    assert [j.attrib["name"] for j in group.findall("joint")] == ARM

def test_joint_limits_have_finite_motion_limits():
    data = yaml.safe_load((CONFIG / "joint_limits.yaml").read_text())
    for name in ARM:
        limits = data["joint_limits"][name]
        assert limits["has_velocity_limits"] and 0.0 < limits["max_velocity"] <= 1.0
        assert limits["has_acceleration_limits"] and 0.0 < limits["max_acceleration"] <= 2.0

def test_controller_joint_order_matches_arm():
    data = yaml.safe_load((CONFIG / "moveit_controllers.yaml").read_text())
    assert data["moveit_simple_controller_manager"]["mecharm_controller"]["joints"] == ARM
```

- [ ] **Step 2: Run the test to verify it fails**

Run from the repository in WSL:

```bash
python3 -m pytest src/mecharm_moveit_config/test/test_moveit_config.py -v
```

Expected: FAIL because the configuration files do not exist.

- [ ] **Step 3: Create the ROS description package**

Use `ament_cmake`; install the existing `urdf` directory without copying meshes:

```cmake
cmake_minimum_required(VERSION 3.8)
project(mycobot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
ament_package()
```

The package manifest must declare name `mycobot_description`, version `0.1.0`, MIT license, `ament_cmake` build tool, and `urdf` plus `xacro` execution dependencies.

- [ ] **Step 4: Create the MoveIt semantic/configuration files**

In `mecharm_270_pi.srdf`, define robot `mecharm_270_pi`, group `arm` with the six exact joints, group state `home` with all six values `0`, end-effector `adaptive_gripper` with parent link `gripper_base`, and virtual joint `world_fixed` from `world` to `base`. Disable only adjacent-link collisions and pairs confirmed always colliding by the model validation command below.

Use these initial planning values:

```yaml
# kinematics.yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.10
  kinematics_solver_attempts: 5
```

```yaml
# joint_limits.yaml (repeat for all six arm joint names)
joint_limits:
  joint1_to_base: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
  joint2_to_joint1: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
  joint3_to_joint2: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
  joint4_to_joint3: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
  joint5_to_joint4: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
  joint6_to_joint5: {has_velocity_limits: true, max_velocity: 0.8, has_acceleration_limits: true, max_acceleration: 1.5}
```

Map `mecharm_controller` to `FollowJointTrajectory`, action namespace `follow_joint_trajectory`, default `true`, and the exact six-joint order. Configure OMPL `RRTConnectkConfigDefault` as the default planner with `planning_time: 5.0` and `planning_attempts: 5`.

- [ ] **Step 5: Create the MoveIt launch file**

Use `MoveItConfigsBuilder("mecharm_270_pi", package_name="mecharm_moveit_config")`, explicitly point `robot_description` to `mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf`, add semantic/kinematics/planning/controller YAML files, and launch `move_group`, `robot_state_publisher`, and RViz only when `use_rviz:=true`.

- [ ] **Step 6: Build and validate**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mycobot_description mecharm_moveit_config
source install/setup.bash
python3 -m pytest src/mecharm_moveit_config/test/test_moveit_config.py -v
xacro "$(ros2 pkg prefix mycobot_description)/share/mycobot_description/urdf/mecharm_270_pi/mecharm_270_pi_adaptive_gripper.urdf" >/tmp/mecharm.urdf
check_urdf /tmp/mecharm.urdf
```

Expected: build and tests PASS; `check_urdf` prints a tree rooted at `base`.

- [ ] **Step 7: Commit**

```bash
git add simulation/urdf/mycobot_description/package.xml simulation/urdf/mycobot_description/CMakeLists.txt src/mecharm_moveit_config
git commit -m "feat: add mechArm MoveIt 2 configuration"
```

### Task 3: Implement trajectory validation and interpolation

**Files:**
- Create: `src/mecharm_pick_place/mecharm_pick_place/trajectory.py`
- Create: `src/mecharm_pick_place/test/test_trajectory.py`

**Interfaces:**
- Consumes: raw joint names and `(positions, time_from_start)` points from a ROS action goal.
- Produces: `normalize_trajectory(joint_names, points, required_names, limits) -> NormalizedTrajectory` and `sample_trajectory(trajectory, elapsed_s) -> tuple[float, ...]`.

- [ ] **Step 1: Write failing pure-Python tests**

```python
import pytest
from mecharm_pick_place.trajectory import RawPoint, normalize_trajectory, sample_trajectory

ARM = ("j1", "j2")
LIMITS = {"j1": (-1.0, 1.0), "j2": (-2.0, 2.0)}

def test_reorders_every_point_and_interpolates_by_timestamp():
    trajectory = normalize_trajectory(
        ("j2", "j1"),
        (RawPoint((0.0, 0.0), 0.0), RawPoint((2.0, 1.0), 2.0)),
        ARM, LIMITS,
    )
    assert trajectory.points[1].positions == (1.0, 2.0)
    assert sample_trajectory(trajectory, 1.0) == pytest.approx((0.5, 1.0))

@pytest.mark.parametrize("names", [("j1",), ("j1", "j1"), ("j1", "bad")])
def test_rejects_missing_duplicate_or_unknown_names(names):
    with pytest.raises(ValueError):
        normalize_trajectory(names, (RawPoint(tuple(0.0 for _ in names), 0.0),), ARM, LIMITS)

def test_rejects_nonincreasing_time_and_limit_violation():
    with pytest.raises(ValueError):
        normalize_trajectory(ARM, (RawPoint((0.0, 0.0), 1.0), RawPoint((0.0, 0.0), 1.0)), ARM, LIMITS)
    with pytest.raises(ValueError):
        normalize_trajectory(ARM, (RawPoint((1.1, 0.0), 0.0),), ARM, LIMITS)
```

- [ ] **Step 2: Run tests and confirm import failure**

Run:

```bash
python3 -m pytest src/mecharm_pick_place/test/test_trajectory.py -v
```

Expected: FAIL with `ModuleNotFoundError: mecharm_pick_place.trajectory`.

- [ ] **Step 3: Implement immutable trajectory types and validation**

```python
from dataclasses import dataclass
from math import isfinite
from bisect import bisect_right

@dataclass(frozen=True)
class RawPoint:
    positions: tuple[float, ...]
    time_from_start: float

@dataclass(frozen=True)
class NormalizedTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[RawPoint, ...]

def normalize_trajectory(joint_names, points, required_names, limits):
    names = tuple(joint_names)
    required = tuple(required_names)
    if len(names) != len(set(names)) or set(names) != set(required):
        raise ValueError("goal must contain each required joint exactly once")
    if not points:
        raise ValueError("trajectory is empty")
    source_index = {name: i for i, name in enumerate(names)}
    normalized = []
    previous_time = -1.0
    for point in points:
        if len(point.positions) != len(names) or not isfinite(point.time_from_start):
            raise ValueError("invalid trajectory point dimensions")
        if point.time_from_start < 0.0 or point.time_from_start <= previous_time:
            raise ValueError("time_from_start must be nonnegative and strictly increasing")
        positions = tuple(float(point.positions[source_index[name]]) for name in required)
        for name, value in zip(required, positions):
            lower, upper = limits[name]
            if not isfinite(value) or not lower <= value <= upper:
                raise ValueError(f"{name} position is outside limits")
        normalized.append(RawPoint(positions, float(point.time_from_start)))
        previous_time = point.time_from_start
    return NormalizedTrajectory(required, tuple(normalized))

def sample_trajectory(trajectory, elapsed_s):
    points = trajectory.points
    if elapsed_s <= points[0].time_from_start:
        return points[0].positions
    if elapsed_s >= points[-1].time_from_start:
        return points[-1].positions
    right = bisect_right([p.time_from_start for p in points], elapsed_s)
    left_point, right_point = points[right - 1], points[right]
    ratio = (elapsed_s - left_point.time_from_start) / (right_point.time_from_start - left_point.time_from_start)
    return tuple(a + ratio * (b - a) for a, b in zip(left_point.positions, right_point.positions))
```

- [ ] **Step 4: Run tests**

Run: `python3 -m pytest src/mecharm_pick_place/test/test_trajectory.py -v`

Expected: all trajectory tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/mecharm_pick_place/mecharm_pick_place/trajectory.py src/mecharm_pick_place/test/test_trajectory.py
git commit -m "feat: validate and sample arm trajectories"
```

### Task 4: Add the FollowJointTrajectory action adapter

**Files:**
- Create: `src/mecharm_pick_place/mecharm_pick_place/trajectory_controller_node.py`
- Create: `src/mecharm_pick_place/test/test_trajectory_controller_policy.py`
- Modify: `src/mecharm_pick_place/package.xml`
- Modify: `src/mecharm_pick_place/setup.py`
- Modify: `config/simulation.yaml`

**Interfaces:**
- Consumes: `/mecharm_controller/follow_joint_trajectory`, `/joint_states`, `/mecharm/gripper_command`.
- Produces: `/mecharm/joint_target`, action feedback/result, and a hold target on cancel/abort.

- [ ] **Step 1: Extract and test terminal-state policy**

Create a ROS-independent helper `execution_outcome(final_error, feedback_age, elapsed, duration, tolerance, stale_timeout, goal_time_tolerance) -> str` returning exactly `running`, `succeeded`, `stale_feedback`, or `goal_tolerance_violated`. Test boundary cases: fresh feedback inside tolerance succeeds; stale feedback aborts; elapsed beyond `duration + goal_time_tolerance` aborts; otherwise it remains running.

Run: `python3 -m pytest src/mecharm_pick_place/test/test_trajectory_controller_policy.py -v`

Expected before helper implementation: FAIL; expected after implementation: PASS.

- [ ] **Step 2: Implement the action server**

The node must declare parameters `publish_rate_hz=120.0`, `joint_tolerance=0.02`, `feedback_stale_timeout_sec=0.5`, `goal_time_tolerance_sec=1.0`, and six URDF limit pairs. Use `ActionServer(..., FollowJointTrajectory, "/mecharm_controller/follow_joint_trajectory", execute_callback=..., goal_callback=..., cancel_callback=...)` with a `ReentrantCallbackGroup` and `MultiThreadedExecutor`.

Convert every `JointTrajectoryPoint` to `RawPoint` using `sec + nanosec * 1e-9`, call `normalize_trajectory`, then run a 120 Hz loop based on the node clock. Publish one `JointState` containing the six sampled arm positions plus the most recently commanded `gripper_controller` position. Build feedback from measured `/joint_states`; cancel and all abort paths publish a single hold command using the latest measured positions before returning.

Use result error codes `INVALID_JOINTS`, `INVALID_GOAL`, `PATH_TOLERANCE_VIOLATED`, `GOAL_TOLERANCE_VIOLATED`, and `SUCCESSFUL` from `FollowJointTrajectory.Result`.

- [ ] **Step 3: Register ROS dependencies and entry point**

Add `control_msgs` and `trajectory_msgs` as execution dependencies, retain `sensor_msgs`, and add:

```python
"isaac_trajectory_controller = mecharm_pick_place.trajectory_controller_node:main",
```

to `console_scripts`.

- [ ] **Step 4: Add node parameters**

Add a top-level `isaac_trajectory_controller.ros__parameters` block to `config/simulation.yaml` with the values from Step 2 and the six joint limits read from the URDF. Keep gripper open/closed physical targets `0.15` and `-0.75` rad.

- [ ] **Step 5: Build and test**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mecharm_pick_place
source install/setup.bash
pytest -q src/mecharm_pick_place/test
ros2 run mecharm_pick_place isaac_trajectory_controller --ros-args --params-file config/simulation.yaml
```

Expected: tests PASS; the last command prints that the action is ready and waits without traceback. Stop it with Ctrl+C.

- [ ] **Step 6: Commit**

```bash
git add src/mecharm_pick_place config/simulation.yaml
git commit -m "feat: add Isaac trajectory action adapter"
```

### Task 5: Replace embedded rclpy with an Isaac ROS 2 OmniGraph

**Files:**
- Create: `simulation/isaac/ros2_graph.py`
- Create: `simulation/isaac/grasp_monitor.py`
- Create: `tests/test_isaac_graph_contract.py`
- Modify: `simulation/isaac/start_simulation.py`
- Delete: `simulation/isaac/mecharm_bridge.py`

**Interfaces:**
- Consumes: `/mecharm/joint_target`, object/gripper state, robot prim `/World/mecharm_270_pi`.
- Produces: `/joint_states`, named articulation targets, and fixed joint `/World/mecharm_grasp_fixed_joint`.

- [ ] **Step 1: Write a source-level graph contract test**

```python
from pathlib import Path

GRAPH = Path("simulation/isaac/ros2_graph.py").read_text(encoding="utf-8")

def test_graph_uses_compiled_bridge_nodes_and_expected_topics():
    for token in ("ROS2PublishJointState", "ROS2SubscribeJointState",
                  "IsaacArticulationController", '"/joint_states"',
                  '"/mecharm/joint_target"'):
        assert token in GRAPH
    assert "import rclpy" not in GRAPH
```

Run: `python -m pytest tests/test_isaac_graph_contract.py -v`

Expected: FAIL because `ros2_graph.py` does not exist.

- [ ] **Step 2: Implement `create_ros2_graph(robot_prim_path)`**

Use `og.Controller.edit` to create `OnPlaybackTick`, `IsaacReadSimulationTime`, `ROS2Context`, `ROS2PublishJointState`, `ROS2SubscribeJointState`, and `IsaacArticulationController`. Connect the tick to publish, subscribe, and controller; connect context and timestamp; connect subscriber `jointNames`, `positionCommand`, `velocityCommand`, and `effortCommand` outputs to the controller. Set publisher target prim to `[usdrt.Sdf.Path(robot_prim_path)]`, publisher topic `/joint_states`, subscriber topic `/mecharm/joint_target`, and controller `robotPath` to the robot prim.

Return the graph path `/ActionGraph` and raise `RuntimeError` if any expected node or attribute is absent after creation.

- [ ] **Step 3: Move physical attachment into `GraspMonitor`**

Give `GraspMonitor` constructor arguments `stage`, `articulation`, `object_prim_path`, `gripper_body_path`, `gripper_joint_name`, `closed_threshold=-0.68`, `open_threshold=0.08`, and `attach_distance=0.12`. Its `update()` reads the actual gripper DOF and world-space distance; on close/in-range it creates the pose-preserving fixed joint, and on open it removes it. Store `localPos0/localRot0/localPos1/localRot1` computed from the current body transforms so attachment cannot snap the cube.

- [ ] **Step 4: Wire the graph and monitor into `start_simulation.py`**

After opening the saved stage, create `SimulationContext(physics_dt=1/120.0, rendering_dt=1/60.0)`, call `create_ros2_graph(ROBOT_PRIM_PATH)`, initialize the articulation and `GraspMonitor`, play, and step until app exit. Call `monitor.update()` after each physics step. Keep `--smoke-test` at 30 steps and print `Isaac ROS 2 graph smoke test completed: 30 simulation steps`.

Remove the `MechArmBridge` import and delete its file only after the new graph smoke test passes.

- [ ] **Step 5: Run static and Isaac smoke tests**

```powershell
python -m pytest tests/test_isaac_graph_contract.py -v
& 'E:\AIRobotic\isaac-sim\python.bat' 'E:\机器人集成小组项目\实验二\simulation\isaac\start_simulation.py' --project-root 'E:\机器人集成小组项目\实验二' --headless --smoke-test
```

Expected: pytest PASS; Isaac exits after 30 frames without `rclpy` publisher crashes, missing graph nodes, or PhysX mimic errors.

- [ ] **Step 6: Verify ROS transport manually**

With Isaac running normally, run in WSL:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 5 ros2 topic echo /joint_states --once
```

Expected: one `JointState` containing all six arm names and `gripper_controller`.

- [ ] **Step 7: Commit**

```bash
git add simulation/isaac/start_simulation.py simulation/isaac/ros2_graph.py simulation/isaac/grasp_monitor.py tests/test_isaac_graph_contract.py
git rm simulation/isaac/mecharm_bridge.py
git commit -m "feat: control Isaac through ROS 2 OmniGraph"
```

### Task 6: Implement the collision-aware fixed pick-and-place node

**Files:**
- Create: `src/mecharm_moveit_demo/package.xml`
- Create: `src/mecharm_moveit_demo/CMakeLists.txt`
- Create: `src/mecharm_moveit_demo/src/fixed_pick_place_moveit.cpp`
- Create: `src/mecharm_moveit_demo/config/fixed_pick_place.yaml`
- Create: `src/mecharm_moveit_demo/test/test_task_parameters.py`

**Interfaces:**
- Consumes: MoveIt group `arm`, `/mecharm_controller/follow_joint_trajectory`, `/mecharm/gripper_command`, and fresh `/joint_states`.
- Produces: one complete home/pick/lift/transfer/place/retreat cycle and process exit code 0 on success, nonzero on failure.

- [ ] **Step 1: Write a failing parameter contract test**

```python
from pathlib import Path
import yaml

def test_fixed_pick_place_parameters_match_isaac_scene():
    data = yaml.safe_load(Path("src/mecharm_moveit_demo/config/fixed_pick_place.yaml").read_text())
    p = data["fixed_pick_place"]["ros__parameters"]
    assert p["pick_xyz"] == [0.18, 0.08, 0.025]
    assert p["place_xyz"] == [0.18, -0.08, 0.025]
    assert p["velocity_scaling"] == 0.20
    assert p["acceleration_scaling"] == 0.20
    assert 0.8 <= p["cartesian_fraction_threshold"] <= 1.0
```

Run: `python3 -m pytest src/mecharm_moveit_demo/test/test_task_parameters.py -v`

Expected: FAIL because the YAML does not exist.

- [ ] **Step 2: Create package and exact task parameters**

Use `ament_cmake` with dependencies `rclcpp`, `moveit_ros_planning_interface`, `geometry_msgs`, `shape_msgs`, `moveit_msgs`, and `std_msgs`. Set parameters: pick/place from Step 1, `pregrasp_clearance=0.08`, `grasp_clearance=0.015`, `retreat_distance=0.08`, `eef_step=0.005`, `jump_threshold=0.0`, `cartesian_fraction_threshold=0.95`, `tool_offset_xyz=[0.0, 0.0, 0.10]`, fixed downward quaternion `[0.0, 1.0, 0.0, 0.0]`, gripper open/closed commands `1.0/0.0`, and motion scaling `0.20/0.20`.

- [ ] **Step 3: Implement reusable checked operations**

In C++, add functions with these signatures:

```cpp
bool execute_pose(moveit::planning_interface::MoveGroupInterface&, const geometry_msgs::msg::Pose&);
bool execute_cartesian(moveit::planning_interface::MoveGroupInterface&, const std::vector<geometry_msgs::msg::Pose>&, double eef_step, double min_fraction);
bool set_gripper(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr&, double command, std::chrono::milliseconds wait);
void add_world_objects(moveit::planning_interface::PlanningSceneInterface&, const Parameters&);
```

`execute_pose` calls `setStartStateToCurrentState`, plans, and executes only on `MoveItErrorCode::SUCCESS`. `execute_cartesian` rejects a fraction below `0.95`, applies iterative parabolic time parameterization at the configured scaling, and executes every resulting point. Every failure returns immediately without launching the next motion.

- [ ] **Step 4: Implement the exact sequence**

Wait up to 5 seconds for a current state; add the table and cube collision objects; move to named state `home`; execute pre-grasp, vertical Cartesian descent, close gripper, attach collision object to `gripper_base`, vertical lift, collision-planned transfer, vertical descent, open gripper, detach/remove the cube, retreat, and return home. Log one stable failure code for each stopped stage and return nonzero.

- [ ] **Step 5: Build and run plan-only validation**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mecharm_moveit_demo
source install/setup.bash
python3 -m pytest src/mecharm_moveit_demo/test/test_task_parameters.py -v
ros2 launch mecharm_moveit_config move_group.launch.py use_rviz:=true
```

Expected: build/test PASS; RViz shows the robot, table, and cube, and a plan to pre-grasp is collision-free. Do not execute until Task 7 transport preflight passes.

- [ ] **Step 6: Commit**

```bash
git add src/mecharm_moveit_demo
git commit -m "feat: add collision-aware MoveIt pick and place"
```

### Task 7: Orchestrate, test, record, and document the simulation

**Files:**
- Create: `src/mecharm_moveit_config/launch/simulation_moveit.launch.py`
- Create: `scripts/start_moveit_sim.ps1`
- Modify: `README.md`
- Modify: `docs/testing.md`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: all packages and Isaac graph from Tasks 1–6.
- Produces: repeatable startup, preflight, one-cycle execution, five-trial acceptance record, and user test instructions.

- [ ] **Step 1: Create the combined WSL launch**

`simulation_moveit.launch.py` must include `move_group.launch.py`, start `isaac_trajectory_controller` with `config/simulation.yaml`, optionally start RViz with `use_rviz`, and start `fixed_pick_place_moveit` only when `run_task:=true`. Default `run_task:=false` so startup cannot move the arm before transport validation.

- [ ] **Step 2: Create the PowerShell launcher**

`start_moveit_sim.ps1` validates the project and Isaac paths, sets `ROS_DOMAIN_ID=0` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, starts Isaac with `scripts/start_isaac.ps1`, and prints—not hides—the exact WSL launch command:

```powershell
wsl.exe -d Ubuntu-22.04 -u xhr -- bash -lc "cd '/mnt/e/机器人集成小组项目/实验二' && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=false"
```

- [ ] **Step 3: Run the no-motion transport preflight**

```bash
ros2 topic echo /joint_states --once
ros2 action list | grep '^/mecharm_controller/follow_joint_trajectory$'
ros2 topic hz /joint_states
```

Expected: one complete joint state, the exact action name, and a stable joint-state rate. Stop `topic hz` after five seconds.

- [ ] **Step 4: Send a synthetic smooth action goal**

With the freshly reset scene still at the defined all-zero `home` state, change only `joint1_to_base` by `+0.05` rad and send two points at 0 and 2 seconds with:

```bash
ros2 action send_goal --feedback /mecharm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [joint1_to_base, joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0}}, {positions: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"
```

Expected: smooth single-joint motion, continuous feedback, and `SUCCESSFUL` result; no oscillation or table contact. If `/joint_states` is not within `0.02` rad of the all-zero home state, reset the Isaac scene before sending this goal.

- [ ] **Step 5: Run one full cycle**

```bash
ros2 launch mecharm_moveit_config simulation_moveit.launch.py use_rviz:=true run_task:=true
```

Expected: the arm completes all twelve specified stages, the cube attaches without snapping, moves to the negative-Y place side, detaches, and the arm returns home.

- [ ] **Step 6: Run the experiment acceptance batch**

Reset the scene before each attempt and run five cycles. Record date/time, planned duration, actual duration, maximum final joint error, attachment result, placement result, and failure code in `results/simulation/<timestamp>/summary.csv`. Acceptance is at least 4 successes out of 5, no joint-limit violation, no table collision, and no visible arm oscillation.

- [ ] **Step 7: Update documentation and ignore generated data**

Document the two-terminal startup, no-motion preflight, manual synthetic goal, one-cycle command, expected topics/action, and shutdown order in `README.md` and `docs/testing.md`. Add generated colcon folders, Python caches, USD scenes, and timestamped result data to `.gitignore`, while keeping `results/**/.gitkeep` and a redacted example summary tracked.

- [ ] **Step 8: Run the final regression**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest src/mecharm_pick_place/test src/mecharm_moveit_config/test src/mecharm_moveit_demo/test -v
```

Expected: zero failed packages and all Python tests PASS.

- [ ] **Step 9: Commit locally; do not push until the user reviews the run**

```bash
git add .gitignore README.md docs/testing.md scripts/start_moveit_sim.ps1 src/mecharm_moveit_config/launch/simulation_moveit.launch.py
git commit -m "docs: add reproducible MoveIt simulation workflow"
git status --short --branch
```

Expected: clean working tree and local branch ahead of `origin/main`; do not run `git push` in this task.
