# Experiment Two Clean Docker Runtime Design

## Goal

Run all ROS 2 Humble, MoveIt 2, trajectory-controller, and fixed pick-and-place nodes in an experiment-specific Docker image while Isaac Sim 5.1 remains native on Windows.

## Architecture

- `mecharm-moveit`: experiment-owned ROS 2 Humble/MoveIt image using Docker Desktop host networking.
- ROS 2 remains container-local; Windows Isaac Sim exchanges named joint states and commands through TCP port 8765.
- The experiment source is bind-mounted read/write; build, install, and log trees live in named Docker volumes.
- RViz is disabled in the first end-to-end run. Isaac Sim provides the visual result. RViz display forwarding is a separate optional follow-up.

## Constraints

- Do not depend on `bobac_fastdds`, `bobac_ros2`, or `bobac_ros2_competition:*`.
- Do not modify or delete experiment-one or competition assets.
- Docker Desktop host networking must be enabled so the container can reach the Windows TCP bridge on localhost.
- Isaac Sim stays at `E:\AIRobotic\isaac-sim` on Windows.
- Do not push until the user verifies `/joint_states` and one complete pick-and-place cycle.
