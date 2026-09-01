# Parallel Gripper Evaluation Design

## Goal

Create an isolated parallel-gripper variant of the mechArm 270 Pi Isaac scene to test whether a simpler official two-slide mechanism produces stable, vertical, centered fixed-point grasping.

## Source and Scope

- Use the three DAE meshes and URDF structure from Elephant Robotics' official `mycobot_ros` parallel-gripper directory.
- Keep the existing adaptive-gripper URDF, scene, scripts, and results unchanged.
- Add a separate parallel-gripper URDF and USD scene rather than replacing the active adaptive scene.

## Architecture

- The parallel gripper has one prismatic master joint and one opposing mimic slide.
- The Isaac import receives finite prismatic limits, lightweight inertias, and simplified non-interlocking collision primitives for stable PhysX motion.
- A dedicated TCP direct-control configuration maps normalized open/close commands to the parallel master range and defines a parallel-gripper TCP at the center between jaws.
- A dedicated direct pick-place launch uses the existing Windows Isaac TCP bridge and container-local ROS 2 runtime, without MoveIt collision planning.

## Acceptance Criteria

- Both jaws translate symmetrically without linkage jitter.
- The jaws are parallel to each other and the approach TCP can align vertically above the cube.
- The cube can be placed between open jaws, attached when closed, transported, released, and returned home.
- The existing adaptive-gripper scene remains runnable without alteration.
