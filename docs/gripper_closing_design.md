# Adaptive Gripper Closing-Range Design

## Goal

Allow the simulated adaptive gripper to close until its two fingertips are nearly touching, while preserving the stable native PhysX mimic-joint linkage.

## Design

- Extend the master `gripper_controller` lower limit from `-0.50 rad` to `-0.75 rad`.
- Apply the same range to mimic joints with multiplier `+1`, and the sign-inverted range to mimic joints with multiplier `-1`.
- Use the same open and closed targets in both the in-app demo and ROS 2 bridge.
- Keep the existing inertias, damping, friction, mimic gearing, and disabled self-collision unchanged.
- Report asynchronous in-app demo failures in the Isaac Sim Console instead of allowing a failed task to appear inactive.

## Success Criteria

- Reimported USD exposes a master range of `[-0.75, 0.15] rad`.
- Every mimic follower has a finite, sign-consistent angular range.
- Empty fingertips nearly meet without visibly crossing through one another.
- Running the in-app script prints either start/completion messages or a Python traceback.

