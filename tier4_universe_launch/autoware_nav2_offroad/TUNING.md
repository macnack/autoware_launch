# Tuning guide: `autoware_nav2_offroad`

How to tune the off-road stack — what each knob does, which file it lives in, and
a symptom-driven workflow. Read the [README](README.md) for the architecture and
[MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md) for the switching logic first.

## The tunable surface

```
goal ─► Nav2 planner ─► Nav2 smoother ─► bridge ─► mode_manager ─► /planning/trajectory ─► controller
          ▲                                (speeds)   (guards)
   costmap (occupancy source)
```

Five groups of parameters, each in its own config file under `config/`:

| Group | File | What it controls |
|-------|------|------------------|
| Nav2 planner / smoother / costmap | `nav2_offroad.param.yaml` | the *path* (shape, clearance, smoothness) |
| Path→trajectory bridge | `nav2_path_to_trajectory_bridge.param.yaml` | the *speed profile* and goal handling |
| Free map | `free_map_publisher.param.yaml` | the synthetic costmap (sim / open ground) |
| Perception occupancy relay | `perception_occupancy_relay.param.yaml` | the real LiDAR-based costmap source |
| Mode manager | node params (set in `nav2_offroad.launch.xml`) | when/how it is safe to switch planners |
| Vehicle command limits | `vehicle_cmd_gate_nav2_offroad.param.yaml` | actuation limits in off-road mode |
| Nav2 MPPI controller (`local_layer:=mppi`) | `nav2_mppi_controller.param.yaml` | the *local control* (critic weights, `vx_*` limits, motion model) |

## How to apply changes

- **Edit the param file, then relaunch.** Most parameters are read once at startup.
- **Live tuning (no restart):** Nav2 servers support runtime parameter updates, which
  is the fastest way to converge planner/smoother/costmap values:

  ```bash
  ros2 param set /planner_server GridBased.tolerance 2.0
  ros2 param set /global_costmap/global_costmap robot_radius 1.0
  ros2 param set /smoother_server simple_smoother.max_its 100
  ```

  Inspect current values with `ros2 param get <node> <param>` or `ros2 param dump <node>`.
- The **bridge**, **free map**, **relay**, and **mode_manager** read parameters at
  startup; change the YAML and relaunch the `nav2_offroad` stack.

## Tune by symptom

| Symptom | First knob to try | File |
|---------|-------------------|------|
| Vehicle drives too fast / too slow off-road | `cruise_speed_mps` | bridge |
| Overshoots or brakes too late at the goal | `goal_taper_distance_m` ↑, `goal_reached_distance_m` | bridge |
| Path hugs obstacles / clips corners | `robot_radius` ↑ | nav2 (costmap) |
| Planner fails to find a path near a goal | `GridBased.tolerance` ↑ | nav2 (planner) |
| Path is jagged / controller oscillates | `simple_smoother.max_its` ↑, `resample_interval_m` ↓ | nav2 / bridge |
| Planner won't go through unobserved space | `allow_unknown: true`, relay `unknown_as_free: true` | nav2 / relay |
| Switch to off-road is rejected ("not continuous") | `max_*_gap` ↑ (carefully) | mode_manager |
| Switch never completes, reverts to on-road | `transition_timeout_s` ↑ | mode_manager |
| Trajectory "lost" → safe-stop too eagerly | `target_trajectory_timeout_s` ↑ | mode_manager |
| Off-road speeds clamped by the gate | `nominal.vel_lim`, accel/jerk limits | vehicle_cmd_gate |

## 1. Nav2 planner / smoother / costmap — `nav2_offroad.param.yaml`

Controls the **geometry** of the path.

**Planner (`GridBased`, NavFn):**

| Param | Default | Effect |
|-------|---------|--------|
| `GridBased.tolerance` | `5.0` | How far (m) from the exact goal a plan may end. Raise if planning fails near obstacles/edges; lower for tighter goal arrival. |
| `GridBased.use_astar` | `true` | A\* (true) vs Dijkstra (false). A\* is faster; Dijkstra can give smoother large-scale paths. |
| `GridBased.allow_unknown` | `true` | Allow routing through unknown cells. Keep `true` for open/off-road; set `false` to forbid unobserved space. |
| `expected_planner_frequency` | `10.0` | Hz the planner expects to be called; mostly affects warnings. |

**Costmap (`global_costmap`):** the biggest safety/clearance knobs.

| Param | Default | Effect |
|-------|---------|--------|
| `robot_radius` | `1.5` | Inflation radius (m). **Most important clearance knob** — raise to keep further from obstacles, lower to fit through gaps. |
| `resolution` | `0.5` | Cell size (m). Smaller = finer paths but more CPU/memory. Match roughly to the occupancy-grid resolution. |
| `track_unknown_space` | `false` | If `true`, unknown is distinct from free (pairs with planner `allow_unknown`). |
| `update_frequency` / `publish_frequency` | `2.0` / `1.0` | How often the costmap refreshes. Raise for faster reaction to a changing perception grid. |
| `always_send_full_costmap` | `true` | Send full costmap each cycle (simpler, heavier). |

**Smoother (`simple_smoother`):**

| Param | Default | Effect |
|-------|---------|--------|
| `simple_smoother.max_its` | `200` | Smoothing iterations. ↑ = smoother but slower; ↓ if `smoothing_timeout_sec` (bridge) is hit. |
| `simple_smoother.tolerance` | `1.0e-10` | Convergence tolerance. Rarely changed. |
| `simple_smoother.do_refinement` | `true` | Extra refinement pass; disable to save time. |

## 2. Path→trajectory bridge — `nav2_path_to_trajectory_bridge.param.yaml`

Turns the Nav2 path into an Autoware `Trajectory` and sets the **speed profile**.

| Param | Default | Effect |
|-------|---------|--------|
| `cruise_speed_mps` | `2.0` | Target speed along the path. The primary speed knob. |
| `goal_taper_distance_m` | `5.0` | Distance before the goal over which speed ramps to 0. ↑ for gentler stops. |
| `goal_reached_distance_m` | `0.8` | Distance at which the goal counts as reached (switches to stop trajectory). |
| `resample_interval_m` | `0.5` | Spacing of output trajectory points. ↓ = denser/smoother for the controller, more points. |
| `min_trajectory_point_distance_m` | `0.2` | Drops points closer than this (de-duplication). |
| `stop_trajectory_min_length_m` | `0.5` | Length of the synthesized stop trajectory. |
| `publish_rate_hz` | `10.0` | Output rate; match the controller's expected input rate. |
| `smoothing_timeout_sec` | `0.5` | Max time to wait for the Nav2 smoother before using the raw path. |
| `action_server_timeout_sec` | `1.0` | How long to wait for the planner/smoother action servers. |
| `force_engage` / `auto_accept_start` | `false` / `false` | Auto-engage / auto-accept-start. Leave `false` on a real vehicle unless you understand the implications. |

Tip: start speed tuning with `cruise_speed_mps` + `goal_taper_distance_m`, then refine
ride quality with `resample_interval_m` and the smoother iterations.

## 3. Occupancy source

Pick the source with the launch arg `occupancy_grid_source` (`free` | `perception`).

**Free map — `free_map_publisher.param.yaml`** (default; sim / open ground, no sensors):

| Param | Default | Effect |
|-------|---------|--------|
| `resolution` | `0.5` | Cell size; keep equal to costmap `resolution`. |
| `width_m` / `height_m` | `400.0` | Map extent. Must cover the planning range from the vehicle. |
| `auto_center_from_odometry` | `true` | Center the map on the vehicle. |
| `publish_rate_hz` | `1.0` | Republish rate (latched anyway). |

**Perception relay — `perception_occupancy_relay.param.yaml`** (real LiDAR-based costmap):

| Param | Default | Effect |
|-------|---------|--------|
| `input_topic` | `/perception/occupancy_grid_map/map` | Source: the Autoware perception OGM (needs ≥1 LiDAR + sensing pipeline). |
| `output_topic` | `/nav2_offroad/costmap/occupancy_grid` | Latched topic the costmap reads; keep aligned with the costmap `map_topic`. |
| `unknown_as_free` | `false` | Map unknown (−1) → free (0). `true` lets the planner cross unobserved space (aggressive); `false` treats unknown as blocked. |

## 4. Mode manager — params in `nav2_offroad.launch.xml`

Governs **when it is safe to switch** between on-road and off-road. These trade
responsiveness against safety — loosen with care on a moving vehicle.

| Param | Default | Effect |
|-------|---------|--------|
| `target_trajectory_timeout_s` | `1.0` | Max age for a source trajectory to count as live. ↑ tolerates slower planners; too high masks a dead planner. |
| `max_position_gap_m` | `2.0` | Allowed XY gap between ego and the target trajectory's first point at handover. |
| `max_yaw_gap_rad` | `0.5` | Allowed heading mismatch at handover. |
| `max_velocity_step_mps` | `1.0` | Allowed speed jump at handover. |
| `transition_timeout_s` | `5.0` | How long to attempt a switch before aborting (revert or safe-stop). |
| `mode_on_startup` | `AW_PLANNING` | Mode after localization is ready. |
| `min_valid_points` | `3` | Minimum trajectory points to be considered valid. |
| `manage_nav2_lifecycle` | `false` in launch | `true` deactivates Nav2 in on-road mode (on-demand, saves CPU); `false` keeps it hot. |
| `publish_rate_hz` | `10.0` | Output / re-evaluation rate. |

Safety note: the continuity guards (`max_*_gap`) exist to prevent a discontinuous
trajectory reaching the controller mid-motion. Prefer fixing the upstream planner
over loosening these. For a stationary bring-up you can bypass them with
`force: true` in a `~/change_mode` call.

## 5. Vehicle command limits — `vehicle_cmd_gate_nav2_offroad.param.yaml`

Relaxed `vehicle_cmd_gate` limits used in `navigation_mode:=nav2_offroad`. If the
vehicle feels capped or jerky at off-road speeds, tune the `nominal` block:
`vel_lim`, `lon_acc_lim_for_lon_vel`, `lon_jerk_lim_for_lon_acc`, and the steering
limits. These are hard actuation limits — set them to the vehicle's real envelope.

## Recommended tuning workflow

1. **Costmap clearance first** — set `resolution` to match your grid, then
   `robot_radius` to the vehicle's footprint + margin. Confirm the path keeps a
   sane distance from obstacles in RViz.
2. **Planner reachability** — raise `GridBased.tolerance` until goals near
   obstacles/edges plan reliably; set `allow_unknown` / relay `unknown_as_free`
   for how you want to treat unobserved space.
3. **Speed profile** — `cruise_speed_mps`, then `goal_taper_distance_m` and
   `goal_reached_distance_m` for clean stops.
4. **Ride quality** — `resample_interval_m` down and smoother `max_its` up until
   the controller tracks smoothly (watch `smoothing_timeout_sec`).
5. **Switching behavior** — tune the mode_manager guards last, only if handovers
   are rejected or too slow; keep them as tight as the vehicle allows.
6. **Actuation envelope** — set the `vehicle_cmd_gate` limits to the real vehicle.

Validate each step in `planning_simulator` before the vehicle; log `~/status`
and the costmap/trajectory topics to a rosbag for review.

## MPPI controller (`local_layer:=mppi`, BACKLOG #11)

`config/nav2_mppi_controller.param.yaml`. Start points (untuned):

- speed/reverse: `FollowPath.vx_max` (2.0, = bridge cruise); `vx_min` is `0.0` (forward-only
  v1 — see note below; do **not** set it negative without also adding gear-sequencing to
  the cmd_vel bridge).
- feasibility: `AckermannConstraints.min_turning_r` (3.5, = planner radius).
- path tracking vs avoidance: raise `PathAlignCritic.cost_weight` to hug the global path;
  raise `ObstaclesCritic.repulsion_weight` / `critical_weight` to push off obstacles.
- if MPPI stalls in a local minimum, raise `ObstaclesCritic.repulsion_weight` and/or
  `PreferForwardCritic.cost_weight`. GPU strongly recommended (lower `batch_size` /
  `controller_frequency` on CPU).

**Forward-only (v1):** `FollowPath.vx_min` is `0.0` — MPPI will not command reverse. To
re-enable reverse, set `vx_min` back to a negative value (e.g. `-0.35`) here in
`nav2_mppi_controller.param.yaml` **and** implement the bridge's gear-by-sign +
stop-before-gear-change sequencing (BACKLOG #11 "Remaining: Reverse driving"); the bridge
currently assumes forward motion and does not switch gear.
