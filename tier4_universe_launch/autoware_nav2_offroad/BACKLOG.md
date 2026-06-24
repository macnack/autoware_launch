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
