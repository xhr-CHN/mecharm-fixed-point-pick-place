# Adaptive Gripper Closing-Range Implementation Plan

**Goal:** Extend the adaptive gripper's closing travel and make in-app demo startup failures visible.

**Architecture:** The URDF remains the source of joint ranges. Isaac Sim's import workaround restores matching finite limits on mimic joints, while both runtime controllers use the same master endpoints.

**Tech Stack:** URDF, Python, Isaac Sim 5.1, USD/PhysX Mimic Joint API

**Spec:** `docs/gripper_closing_design.md`

## Global Constraints

- Change only Experiment 2.
- Do not run the simulation automatically.
- Do not upload to GitHub until simulation integration is complete.

### Task 1: Synchronize gripper travel

- [x] Change the URDF master and follower limits to the new range.
- [x] Change the Isaac 5.1 imported mimic-limit restoration values.
- [x] Change the ROS 2 and in-app closed targets.

### Task 2: Surface in-app execution failures

- [x] Add an asynchronous task completion callback that prints tracebacks.
- [x] Print a message if the demo is already running.

### Task 3: Static verification

- [x] Parse the URDF as XML.
- [x] Compile the modified Python files without launching Isaac Sim.
- [ ] Rebuild the USD manually and visually confirm the final fingertip gap.
