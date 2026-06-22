# Debugging guide: `autoware_nav2_offroad`

Observability for bringing the off-road stack up on a real vehicle: what to look
at live, how to see *why* a mode switch happened (or didn't), and how to capture
a drive for offline analysis.

## Topics published by `trajectory_mode_manager`

| Topic | Type | Use |
|-------|------|-----|
| `~/status` | `TrajectoryModeState` (latched) | current/requested mode, transition, active route, fault reason |
| `~/debug` | `TrajectoryModeDebug` | the guard inputs: position/yaw/velocity gaps vs limits, source ages, usability, continuity result |
| `~/events` | `std_msgs/String` (latched, depth 10) | one message per mode/transition/fault change (also mirrored to the node log) |
| `~/markers` | `visualization_msgs/MarkerArray` | RViz: mode text overlay at the ego + the continuity-gap line |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | OK / WARN (transition or standby) / ERROR (safe stop), via the diagnostic graph |

(The launch remaps these under `/nav2_offroad/mode_manager` via the
`mode_manager_prefix` arg; e.g. `/nav2_offroad/mode_manager/status`.)

## 1. Visualizations (RViz)

- **Mode overlay + continuity gap** — add a `MarkerArray` display on
  `/nav2_offroad/mode_manager/markers`. You get a text label at the vehicle
  (mode / route / fault) and a line from the ego to the target trajectory's
  first point, **green when the continuity guard passes, red when it fails** —
  the quickest way to see why a switch is blocked.
- **Trajectories** — display `/planning/trajectory` (committed output),
  `/planning/trajectory_pre_mux` (on-road), and `/nav2_offroad/planning/trajectory`
  (off-road) to compare the two planners.
- **Costmap** — display the `OccupancyGrid` on `/nav2_offroad/costmap/occupancy_grid`
  (the free map or the relayed perception grid) plus Nav2's own costmap.

## 2. State / GUI panel

`autoware_nav2_offroad_rviz_plugin` → **OffroadModePanel** (Panels → Add New Panel).
It shows the live mode + transition + fault from `~/status`, and the guard values
from `~/debug` (each gap shown as `value / limit  OK|X`, plus source ages and
liveness), with the OFF-ROAD / ON-ROAD buttons. Use it to see at a glance whether
a switch will be accepted before you press the button.

## 3. Logger / event stream

Every mode change, transition, abort, and fault is:

- printed to the node log (`RCLCPP_INFO`, or `WARN` for faults / safe-stop), and
- published on `~/events` (latched) as a human-readable string.

Tail it live:

```bash
ros2 topic echo /nav2_offroad/mode_manager/events
```

## 4. Record a drive for offline analysis

```bash
ros2 launch autoware_nav2_offroad debug_record.launch.xml
# custom location / extra topics:
ros2 launch autoware_nav2_offroad debug_record.launch.xml \
  bag_uri:=/tmp/offroad_run extra_topics:="/tf /tf_static"
```

Records status / debug / events / markers, both planners' trajectories, the goal,
the costmap occupancy grid, odometry, and `/diagnostics`. Replay with
`ros2 bag play <bag_uri>` and inspect in RViz / `ros2 topic echo`.

## Quick "why didn't it switch?" checklist

1. `ros2 topic echo ~/debug` — is the target source `*_usable` true and fresh
   (`*_age_s` < `target_trajectory_timeout_s`)? If not, the planner/bridge is the
   problem, not the manager.
2. Is `continuity_ok` false? Check which gap exceeds its limit (`position_gap_m`,
   `yaw_gap_rad`, `velocity_gap_mps`) — the red marker line shows the position gap.
   Tune the limits per [TUNING.md](TUNING.md) or fix the upstream trajectory.
3. `ros2 topic echo ~/events` / node log — did it abort on `transition timed out`?
   Raise `transition_timeout_s` or fix why the target never became valid.
4. For a stationary bring-up only, bypass the guards with `force: true` in the
   `~/change_mode` request.
