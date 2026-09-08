# Experiment 2: Fixed-Point Robotic Arm Pick and Place

This directory contains the fixed-point pick-and-place experiment for the mechArm 270 Pi with the adaptive gripper. Simulation runs on Windows with Isaac Sim 5.1.0. ROS 2 Humble and the task nodes run in the dedicated Docker container.

The project includes:

- Elephant Robotics mechArm 270 Pi adaptive-gripper URDF and meshes;
- the `mecharm_pick_place` ROS 2 Python package;
- calibrated fixed-point and Cartesian pick-and-place flows;
- alternating five-cycle pick-and-place and empty-grasp exception tests;
- the Isaac Sim scene, TCP joint bridge, and startup scripts;
- adaptive-gripper loop-joint and drive configuration;
- unit tests and ROS 2 launch files;
- MoveIt 2 configuration for later collision-planning calibration.

## Directory layout

```text
config/                  Experiment configuration
docs/                    Design notes, test records, and videos
scripts/                 Windows startup and environment scripts
simulation/              Isaac Sim scripts, scene, URDF, and meshes
src/mecharm_pick_place/  ROS 2 package
tests/                   Tests that do not require a ROS runtime
results/                 Runtime results and logs
report/                  Experiment report
```

## Startup

Close proxy TUN mode before starting. If TUN was just disabled, restart WSL once:

```powershell
wsl --shutdown
```

Start Docker MoveIt and Isaac Sim from Windows PowerShell:

```powershell
cd "E:\机器人集成小组项目\实验二"
.\scripts\start_moveit_sim.ps1
```

The script builds the dedicated `mecharm-exp2-ros2:humble` image, starts the `moveit` service, launches Isaac Sim, and prints the commands for building and running the ROS package. Wait until the Isaac scene is fully loaded and the timeline is playing.

The Windows Isaac process communicates with the container through the project TCP bridge on port `8765`; runtime ROS 2 discovery does not cross between Windows and the container.

## Build the ROS package

After the container is `Up`, build the package:

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && colcon --log-base /opt/mecharm_ws/log build --base-paths /workspace/mecharm_exp2/simulation/urdf/mycobot_description /workspace/mecharm_exp2/src --build-base /opt/mecharm_ws/build --install-base /opt/mecharm_ws/install --symlink-install"
```

## Cartesian vertical pick and place

This is the recommended single-cycle flow. It uses numerical IK, a vertical tool orientation, the configured grasp coordinates, and separate normal and vertical speeds.

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && ros2 launch mecharm_pick_place cartesian_direct_pick_place.launch.py project_root:=/workspace/mecharm_exp2"
```

The sequence is: open the gripper, move to HOME, approach above the object, descend, close, lift, move to the placement point, release, and return HOME.

## Five alternating cycles

The alternating entry point performs A→B, B→A, A→B, B→A, and A→B. Each cycle returns to HOME before the next one.

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && ros2 launch mecharm_pick_place alternating_cartesian_pick_place.launch.py project_root:=/workspace/mecharm_exp2 cycles:=5"
```

## Empty-grasp exception test

This test closes the gripper at an empty reachable point, reports `NO_OBJECT`, opens the gripper, and returns HOME:

```powershell
docker compose exec moveit bash -lc "source /opt/ros/humble/setup.bash && source /opt/mecharm_ws/install/setup.bash && ros2 launch mecharm_pick_place empty_grasp_test.launch.py project_root:=/workspace/mecharm_exp2"
```

Expected messages include:

```text
EMPTY_GRASP_EXCEPTION code=NO_OBJECT
EMPTY_GRASP_RETURN_HOME
```

## Parameters and scene rebuild

Cartesian coordinates and gripper timing are in `src/mecharm_pick_place/config/direct_cartesian_pick_place.yaml`. Normal arm speed and vertical-stage speed are in `src/mecharm_pick_place/mecharm_pick_place/direct_pick_place_demo.py`.

After changing Isaac model, stiffness, damping, or joint-constraint settings, rebuild the USD scene:

```powershell
.\scripts\start_isaac.ps1 -ProjectRoot "E:\机器人集成小组项目\实验二" -RebuildScene -BuildSceneOnly
```

The current Isaac import defaults are arm stiffness `10000` and damping `3000`. Adaptive-gripper driven joints use stiffness `150`, damping `60`, and maximum force `10`.

## Recordings

Experiment recordings are organized under [`docs/videos/`](docs/videos/). MP4 files are tracked with Git LFS because two recordings exceed GitHub's regular-file size limit.

## Model source

The model is based on the Humble branch of Elephant Robotics' official repository:

```text
https://github.com/elephantrobotics/mycobot_ros2
mycobot_description/urdf/mecharm_270_pi
mycobot_description/urdf/adaptive_gripper
```

The license is stored in `simulation/urdf/mycobot_description/LICENSE`.
