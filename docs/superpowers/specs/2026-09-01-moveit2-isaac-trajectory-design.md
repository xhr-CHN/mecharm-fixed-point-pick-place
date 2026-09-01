# MoveIt 2 Competition-Style Isaac Sim Control Design

## Objective

Reuse the competition control architecture for Experiment 2 while keeping the immediate scope limited to simulation. ROS 2 Humble and MoveIt 2 run in WSL 2. Isaac Sim 5.1 runs on Windows and executes standard time-parameterized arm trajectories. Jetson deployment and Docker packaging are explicitly outside this implementation phase.

## Current Constraints

- Isaac Sim 5.1.0 runs on Windows at `E:\AIRobotic\isaac-sim`.
- ROS 2 Humble runs in the `Ubuntu-22.04` WSL distribution.
- MoveIt 2, `moveit_configs_utils`, and `joint_trajectory_controller` are not currently installed in WSL.
- The existing Isaac-side embedded `rclpy` bridge exits while creating ROS publishers, so it is not used for the primary transport.
- The existing in-app demo proves that the articulation, adaptive-gripper mimic chain, and 120 Hz PD tracking can work, but its arm waypoints are not collision-planned.
- The target cube starts at `(0.18, 0.08, 0.025)` m and the nominal place side is centered at `(0.18, -0.08)` m.

## Selected Architecture

The implementation uses the same separation as the competition system: MoveIt plans complete collision-aware paths, a `FollowJointTrajectory` controller executes every time-stamped waypoint, and the simulator only handles articulation physics.

```text
WSL 2 / ROS 2 Humble
  fixed_pick_place_moveit
    -> MoveIt move_group / OMPL / KDL
    -> /mecharm_controller/follow_joint_trajectory
  isaac_trajectory_controller
    -> validates and executes FollowJointTrajectory goals
    -> publishes /mecharm/joint_target as sensor_msgs/JointState
    <- subscribes /joint_states for feedback

Windows / Isaac Sim 5.1
  programmatically-created ROS 2 OmniGraph
    <- /mecharm/joint_target
    -> Articulation Controller
    -> /joint_states
  physics-step grasp monitor
    -> create/remove USD FixedJoint from measured gripper state and distance
```

The WSL trajectory controller is a simulation adapter. A future real-robot deployment can replace it with the mechArm hardware driver while preserving the MoveIt configuration, action name, task sequence, and trajectory semantics.

## Components

### `mecharm_moveit_config`

A new ROS 2 package provides:

- robot description based on the checked-in mechArm 270 Pi URDF;
- an `arm` planning group containing the six arm joints;
- `gripper_base` as the planning end-effector link;
- joint limits with finite velocity and acceleration values suitable for planning;
- KDL inverse kinematics configuration;
- OMPL planning configuration;
- controller mapping to `/mecharm_controller/follow_joint_trajectory`;
- disabled self-collision pairs generated from the actual model;
- launch files for `move_group`, `robot_state_publisher`, and optional RViz 2.

The mimic followers are excluded from the MoveIt arm group. The gripper master remains independently commanded as `gripper_controller`.

### `isaac_trajectory_controller`

A ROS 2 Python node in the existing `mecharm_pick_place` package implements `control_msgs/action/FollowJointTrajectory` at:

```text
/mecharm_controller/follow_joint_trajectory
```

It accepts exactly the six arm joint names, permits any input order, reorders positions to the model order, and rejects:

- unknown, missing, or duplicate joints;
- non-finite positions;
- positions outside URDF limits;
- empty trajectories;
- non-increasing `time_from_start` values.

During execution it interpolates between the supplied MoveIt waypoints using their timestamps and publishes the complete arm target at 120 Hz on `/mecharm/joint_target`. It subscribes to `/joint_states`, returns action feedback from measured positions, succeeds only within the configured tolerance, and aborts on stale feedback or timeout.

The same node accepts the existing `/mecharm/gripper_command` `Float64` topic and republishes the `gripper_controller` target through `/mecharm/joint_target`, keeping gripper control separate from the six-axis MoveIt trajectory.

### Isaac ROS 2 OmniGraph

The Isaac setup script creates the ROS 2 graph in code; no manual node dragging is required. The graph:

- publishes the articulation joint state on `/joint_states`;
- subscribes to `/mecharm/joint_target` as `sensor_msgs/JointState`;
- applies named joint targets to `/World/mecharm_270_pi`;
- uses ROS domain and middleware settings shared with WSL.

This path uses the compiled Isaac ROS 2 Bridge nodes rather than constructing an embedded Python `rclpy` node.

An Isaac-side Python physics callback monitors the commanded and measured `gripper_controller` position. When the gripper reaches the closed threshold and the cube is within the configured distance, it creates the pose-preserving USD fixed joint. When the gripper reaches the open threshold, it removes that joint. This callback has no ROS client; it only observes the articulation and stage that are already controlled by OmniGraph.

### `fixed_pick_place_moveit`

A C++ MoveIt 2 node reproduces the competition flow using `MoveGroupInterface`:

1. receive fresh joint state;
2. add the work table and target cube to the MoveIt planning scene;
3. plan and execute a home motion;
4. plan to a pre-grasp pose above the cube;
5. plan a vertical Cartesian descent to the grasp pose;
6. close the adaptive gripper;
7. wait for measured gripper completion, then attach the cube in the MoveIt planning scene while the Isaac grasp monitor creates the physical connection;
8. plan a vertical lift;
9. plan a collision-free transfer to the place side;
10. plan a vertical descent;
11. open the gripper and detach the cube;
12. retreat and return home.

Velocity and acceleration scaling default to `0.20`, matching the competition configuration. Every MoveIt waypoint is preserved; the implementation does not replace a planned path with a straight interpolation between only its endpoints.

## Coordinate and Pose Rules

- `base` is the MoveIt planning frame and must match the Isaac robot base frame.
- Pick and place coordinates describe the cube center on the table.
- Pre-grasp and retreat poses are generated by adding a positive Z clearance.
- Grasp poses are computed for the actual tool frame and include the measured offset from `gripper_base` to the fingertip center.
- Tool orientation remains downward and fixed during vertical approach, lift, and release.
- The planning scene contains the same table dimensions and pose as Isaac Sim.

The fingertip tool offset is calibrated once from the USD/URDF model and stored as a named parameter. It is not hidden inside hard-coded joint angles.

## Object Attachment

The Isaac grasp monitor creates a fixed joint only after the gripper closes and the object is within the configured attachment distance. The fixed joint preserves the current relative transform, preventing the object from snapping to the gripper origin. MoveIt attaches the matching collision object to `gripper_base` for transfer planning and removes it after release. Opening the gripper removes the Isaac joint before MoveIt removes the attached collision object.

## Failure Handling

- Planning failure: do not execute; publish a failed attempt result.
- Cartesian path fraction below the acceptance threshold: do not execute.
- Trajectory validation failure: abort the action before publishing any target.
- Missing or stale `/joint_states`: abort and hold the most recent safe target.
- Execution timeout or excessive final joint error: abort the action and stop the task sequence.
- Object outside attachment range: leave it detached and fail the attempt.
- ROS 2 transport unavailable: the startup smoke test fails before arm motion is enabled.

No automatic retry may move the arm until the current action is terminal and the measured joint state is fresh.

## Verification

### Static checks

- URDF and SRDF parse successfully.
- MoveIt Setup Assistant/config validation recognizes six arm joints and the expected end-effector link.
- Python and C++ packages build with `colcon`.
- Unit tests cover trajectory validation, joint-name reordering, interpolation, timeout, and final-tolerance behavior.

### Transport checks

- Isaac publishes all expected joint names on `/joint_states`.
- A synthetic two-point `FollowJointTrajectory` goal moves one arm joint smoothly and returns success.
- Canceling a goal holds the latest measured pose without starting another trajectory.

### Planning checks

- RViz 2 plan-only mode shows the table and cube collision objects.
- Home-to-pre-grasp and transfer paths are collision-free.
- The Cartesian approach, lift, descent, and retreat meet the configured fraction threshold.

### End-to-end acceptance

- Isaac completes one full pick-and-place cycle without joint-limit violations, table collision, visible oscillation, or object snapping.
- Five consecutive attempts complete with at least four successes, matching the experiment requirement.
- Recorded results include planned trajectory timestamps, measured joint states, action result, and failure code.

## Migration Boundary

This phase ends when the Isaac simulation passes the acceptance criteria. Docker, Jetson, and the real mechArm driver are not added now. The standard `FollowJointTrajectory` boundary is intentionally retained so that future migration changes only the execution adapter.
