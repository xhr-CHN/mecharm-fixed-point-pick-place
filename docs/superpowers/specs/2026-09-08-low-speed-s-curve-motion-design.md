# Low-speed S-curve motion design

## Goal

Reduce visible arm vibration during the existing direct pick-and-place sequence without changing grasp coordinates, gripper timing, or Isaac Sim drive parameters.

## Current behavior

The direct executor publishes interpolated joint targets at 120 Hz. Motion is already divided into small command increments, but it uses a cubic smoothstep profile and allows arm motion up to 12 degrees per second. The cubic profile has zero endpoint velocity but nonzero endpoint acceleration, which can excite the simulated joints when a stage starts or finishes.

## Selected design

1. Replace the cubic interpolation profile with a quintic S-curve: `6t^5 - 15t^4 + 10t^3`.
2. Reduce the direct pick-and-place arm speed limit from 12 degrees per second to 4 degrees per second.
3. Keep continuous 120 Hz command publication. Do not insert stops between artificial one-centimeter segments because repeated acceleration and deceleration would add disturbance.
4. Preserve all joint waypoints, Cartesian targets, gripper speed, gripper waits, stiffness, damping, and scene settings.

## Affected files

- `src/mecharm_pick_place/mecharm_pick_place/direct_motion_probe.py`: use quintic S-curve interpolation for every direct motion segment.
- `src/mecharm_pick_place/mecharm_pick_place/direct_pick_place_demo.py`: lower the maximum arm speed to 4 degrees per second.
- Relevant tests: verify endpoint values and the zero-velocity/zero-acceleration properties of the interpolation curve, plus the configured speed limit.

## Success criteria

- Existing unit and contract tests pass.
- The package builds successfully in the experiment-two Docker container.
- The complete direct pick-and-place sequence still reaches all existing stages and reports `DIRECT_PICK_PLACE_SUCCESS`.
- Visual vibration during travel and stage transitions is lower than with the 12-degree-per-second cubic profile.

## Risks and rollback

The complete cycle will take longer. If low-frequency sag or looseness becomes more visible, the vibration is primarily caused by Isaac dynamics rather than trajectory sampling; in that case restore the previous speed/profile and diagnose the drive settings separately.
