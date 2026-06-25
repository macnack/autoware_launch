# autoware_nav2_offroad_msgs

Interface definitions (messages and services) for the
[`autoware_nav2_offroad`](../autoware_nav2_offroad) trajectory mode management.

Split into its own package so that consumers (HMI, AD API adaptors, tests) can
depend on the interface without pulling in the off-road runtime nodes.

## Contents

| Interface | Purpose |
|-----------|---------|
| `msg/TrajectoryModeState` | Status published by `trajectory_mode_manager` on `~/status` (current/requested mode, transition, active planner, fault reason) |
| `srv/ChangeTrajectoryMode` | Request a mode change (`target_mode`, `force`); returns acceptance + resulting mode |

See [`autoware_nav2_offroad/MODE_MANAGER_DESIGN.md`](../autoware_nav2_offroad/MODE_MANAGER_DESIGN.md)
for how these are used.
