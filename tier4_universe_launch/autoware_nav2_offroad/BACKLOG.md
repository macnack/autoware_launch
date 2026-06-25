# autoware_nav2_offroad — backlog

Known follow-ups / future work. Not blocking the current (validated) off-road
sim flow.

## 1. Goal-heading-aware planning (SmacPlannerHybrid) — DONE (branch feat/offroad-smac-planner)

Implemented: planner switched to `nav2_smac_planner/SmacPlannerHybrid` (Dubin,
minimum_turning_radius ~3.5 m), inflation layer added to the costmap, and the
bridge pins the final trajectory point to the goal heading. `nav2_smac_planner`
added to `package.xml`. Runtime heading verification (drive in sim) still pending.

**Problem:** the current Nav2 planner is `NavfnPlanner` (`GridBased`), a 2-D
holonomic grid planner. It plans to the goal **position only and ignores the goal
heading** — the controller reaches the goal point but not the requested
orientation. NavFn paths are also not kinematically constrained (can demand turns
a car can't make). Additionally, the bridge's `trajectory_builder` derives point
orientation from the path tangent, so a goal heading wouldn't survive downstream.

**Fix:** switch the planner to **`SmacPlannerHybrid`** (Hybrid-A\*):

- respects the goal **pose** (heading), not just position
- nonholonomic / car-like: uses a minimum turning radius (Dubins / Reeds-Shepp),
  so paths are drivable
- its path curves into the goal heading, so the bridge's tangent-based orientation
  then naturally ends at the right heading

**Tasks:**

- add `nav2_smac_planner` (`ros-$ROS_DISTRO-nav2-smac-planner`) to the package
  `package.xml` deps
- replace the `GridBased`/`NavfnPlanner` block in `config/nav2_offroad.param.yaml`
  with a `SmacPlannerHybrid` config: motion model (Ackermann/Dubins),
  `minimum_turning_radius` derived from `wheel_base` / `max_steer_angle`,
  `analytic_expansion_*`, goal heading tolerance. Keep NavFn available as a
  selectable fallback.
- (belt-and-braces) optionally pin the final trajectory point in the bridge to the
  exact goal heading.

## 2. Nav2 autostart vs. localization race — proper fix

`lifecycle_manager_navigation` `autostart:=true` activates `planner_server` before
localization is initialized, so the global_costmap can't get `map→base_link` and
bringup can abort. Current workaround: the **Restart Nav2 stack** button in
`OffroadModePanel`. Proper fix: set `autostart:=false` and trigger STARTUP once
localization is ready (e.g. from the bridge when the first ego pose arrives, or a
small helper gated on `/localization/kinematic_state`).

## 3. Off-road trajectory validation

In off-road mode the route-gated `planning_validator` is bypassed (a relay sends
the smoothed trajectory straight to `/planning/trajectory`). There is currently no
off-road-specific safety validation. Repurpose the `trajectory_mode_manager`
continuity guards (`evaluateGuards`) + safe-stop as a lightweight off-road
validator, or add an off-road profile to `planning_validator` that does not require
a lanelet route.

## 4. Real-vehicle costmap

`occupancy_grid_source:=perception` relays the Autoware perception OGM, which needs
a LiDAR + sensing pipeline (absent in `planning_simulation`). Validate on a vehicle
/ a sim that publishes a realistic occupancy grid; tune `robot_radius` and costmap
layers for real obstacles (currently global costmap + static layer only — no local
costmap / dynamic-obstacle avoidance).

## 5. Upstream / release

The work spans three repos on feature branches (`autoware_internal_msgs`,
`autoware_universe`, `autoware_launch`). To ship: merge upstream, cut tags, and bump
`repositories/autoware.repos`. Until then nav2 deps come via rosdep from the
package `package.xml`.

## 7. Reduce final-goal HEADING error (priority: heading, x/y can be off)

Measured with `scripts/offroad_goal_scenario.py` (goal at ego+20 m, yaw 90 deg):
position ~0.66-0.75 m, **heading ~21-37 deg** with high run-to-run variation.
Tightening the off-road params (`goal_reached_distance_m` 0.8->0.3,
`tolerance` 0.5->0.25) did not help — the vehicle stops ~0.66 m short of the
trajectory end (controller terminal accuracy), and at that point it is still
mid-rotation toward the goal heading.

**Heading is the metric that matters** (position can be off). Root cause: the
`trajectory_builder` recomputes each point's orientation from the **path tangent**
and only pins the very last point to the goal heading — a single-point jump the
controller ignores. So the commanded heading near the goal lags the goal heading.

**Implemented:** the `trajectory_builder` now blends point orientation from the
path tangent to the goal heading over `goal_heading_blend_distance_m` (default
4 m) before the goal, so the controller rotates into the goal heading along the
approach instead of seeing a single end-point jump.

**Measured (sim, SmacPlannerHybrid + 4 m blend):**

- position error: **reliably** down from ~0.66 m to **~0.28 m**
- heading error: improved on average but **high variance** (~14-31 deg run to
  run) — NOT reliably small

**Why heading still varies:** a forward-only (DUBIN) car arriving at a point goal
can only realise the heading its planned path geometry delivers; when the
controller decelerates and stops a little short, or the planned path doesn't fully
curve to the goal pose that run, the final heading lags. The blend biases the
command toward the goal heading but cannot make the vehicle physically rotate
without the path curving there.

**To make goal heading reliable (next):**

- enable **REEDS_SHEPP** (item 6) so Hybrid-A\* can plan a real maneuver
  (back-up / S-curve) that physically arrives at the goal heading — the single
  biggest lever for tight/perpendicular goal headings
- tune the `trajectory_follower` terminal behaviour so it tracks the final
  heading-aligning curve instead of stopping short
- increase `goal_heading_blend_distance_m` (more approach heading authority) and
  the planner `analytic_expansion_*` so the path ends with a longer aligned segment
- average several scenario runs (single runs are noisy)

## 8. Automatic comms-loss watchdog for Return-To-Home

**Trigger:** no heartbeat message received on a configurable topic within a configurable timeout → automatically call `return_home` + engage the vehicle autonomously.

**What needs building:**

- `return_home_node` parameter: `watchdog_topic` (string, e.g. `/ground_station/heartbeat`), `watchdog_timeout_sec` (float, e.g. 5.0), `watchdog_enabled` (bool, default false so v1 is manual-only).
- On each heartbeat message the node resets a timer; on expiry it calls its own internal `return_home` logic and additionally sends an `EngageCommand` to the Autoware API to engage autonomous mode (or publishes to the configured engage topic).
- The operator must have called `set_home` before the watchdog fires; if no home is set the watchdog logs a fatal error but does not trigger (safe default).
- Expose `watchdog_armed` status in `ReturnHomeState` so operators can monitor it in the RViz panel.

**Cross-reference:** once the vehicle is autonomously returning, dynamic-obstacle avoidance during the return is the same open problem as item 4 (real-vehicle costmap / local costmap with obstacle layers) — those two items should be addressed together.

## 6. Reverse / Reeds-Shepp support

> **Prioritized next step** — this is the agreed follow-up to drive down the
> final-goal **heading** error (item 7). The 4 m heading blend reliably fixed
> position (~0.28 m) but heading stays high-variance (14-31 deg) because a
> forward-only DUBIN car can't physically realise a tight/perpendicular goal
> heading. REEDS_SHEPP lets the planner do a real maneuver (back-up / S-curve)
> that arrives at the goal pose. Plan: **TDD the builder's reverse/cusp handling
> first** (that is where the correctness risk is), on a new branch.

The planner is `SmacPlannerHybrid` with `motion_model_for_search: DUBIN`
(forward-only). Switching to `REEDS_SHEPP` lets Hybrid-A\* plan reverse segments
(reach tight goal headings, back-up maneuvers) — but the planner flag alone is not
enough: the bridge/`trajectory_builder` currently assumes pure forward motion, so
reverse segments would be driven forward (wrong direction + 180°-wrong heading).

**Tasks (the real work is the bridge, not the planner):**

- planner: set `motion_model_for_search: REEDS_SHEPP` (and tune `reverse_penalty`).
- `trajectory_builder`:
  - detect per-segment motion direction (path tangent vs pose orientation, or the
    planner's direction info)
  - set **negative** `longitudinal_velocity_mps` on reverse segments
  - set point orientation to the **vehicle heading** (opposite the motion tangent
    while reversing), not the motion tangent
  - insert a **zero-velocity point at each cusp** (forward↔reverse switch) so the
    controller can change direction
- bridge: command `GearCommand::REVERSE` on reverse segments (gear must match the
  current segment's direction; currently always `DRIVE`)
- verify `trajectory_follower` tracks the reverse trajectory; relaxed
  `vehicle_cmd_gate` limits already allow it.

This is a feature, not a config flip — TDD the builder's reverse/cusp handling.

## 11. Adopt the Nav2 MPPI controller as the off-road local layer

Deep-research finding (see
[research/lattice-motion-planning.md](research/lattice-motion-planning.md)): the
highest-value planner change for off-road is **not** swapping the global planner but
adding **`nav2_mppi_controller`** underneath it. MPPI (Model Predictive Path Integral)
is what DARPA RACER / NASA JPL / Georgia Tech AutoRally use for rough-terrain
kinodynamic driving — it samples control sequences forward through a dynamics model
(feasible by construction), handles traction/slope/contact dynamics and reactive
avoidance the geometric global planner ignores, reverses by default (`vx_min` −0.35),
and is cost-shapeable so the traversability costmap (item 4) flows straight into
control. State lattice (`SmacPlannerLattice`) is at most an optional A/B test.

Tasks:

- bring up `nav2_mppi_controller` in the off-road stack (Ackermann motion model),
  replacing/augmenting the current `trajectory_follower`-driven control where Nav2
  controls (overlaps with the teach&repeat cmd_vel→gate routing);
- add an Obstacles/dynamic-obstacle critic (vanilla MPPI does not model dynamic
  obstacles) and tune critics (PathAlign / PathFollow / PreferForward / Cost);
- budget a GPU (parallel rollouts are the cost); watch for local minima (recent
  repulsive-potential fixes exist);
- keep Hybrid A* (REEDS_SHEPP) as the global planner: it plans the route, MPPI drives it.

Research also confirms REEDS_SHEPP as the right priority and advises **against**
a custom state-lattice / spatiotemporal-conformal-lattice / RRT* / end-to-end learned
planner; `SmacPlannerLattice` is at most an optional A/B test.

**Implemented (config + launch, branch `feat/offroad-mppi-local-layer`):**

- `local_layer:=mppi` brings up `controller_server` (`nav2_mppi_controller`, Ackermann,
  `vx_min:-0.35` reverse, critics incl. **ObstaclesCritic**) + a rolling `local_costmap` +
  `bt_navigator` (NavigateToPose orchestration), all under the existing lifecycle manager;
  the path→trajectory bridge is suppressed in this mode. Default stays `local_layer:=bridge`.
- `global_planner:=lattice` overlays `SmacPlannerLattice` as the optional A/B planner
  (Hybrid-A\* remains default).
- Config: `config/nav2_mppi_controller.param.yaml`, `config/nav2_bt_navigator.param.yaml`,
  `config/nav2_smac_lattice.param.yaml`. Deps added to `package.xml`.

**Remaining (NOT in this branch):**

- **cmd_vel → gate routing** (shared with the teach&repeat cmd_vel→gate work): MPPI emits
  `/cmd_vel`, but it is not yet routed to `vehicle_cmd_gate`, so `mppi` mode is **not yet
  drivable** end-to-end.
- a goal relay turning `/planning/offroad_goal` into a `NavigateToPose` action goal;
- add a dynamic-obstacle critic and tune critics; sim-drive + **GPU** provisioning; watch
  for local minima;
- keep Hybrid A\* (REEDS_SHEPP) as the global planner: it plans the route, MPPI drives it.
