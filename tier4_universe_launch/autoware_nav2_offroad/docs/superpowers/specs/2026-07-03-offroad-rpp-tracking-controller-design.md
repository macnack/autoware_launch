# RPP tracking controller — split-pipeline architecture for the off-road local layer

**Status:** Design approved (user-confirmed 2026-07-03)
**Date:** 2026-07-03
**Package:** `autoware_nav2_offroad`
**Author:** Maciej Krupka
**Branch / PR:** `feat/offroad-mppi-drive` / PR #7

> **Confirmed decisions (2026-07-03):**
> 1. New launch arg **`local_controller`** (`mppi` default | `rpp`), honored when
>    `local_layer` ∈ {`mppi`, `mppi_recovery`}. Default keeps today's behavior unchanged.
> 2. RPP reverse support ties to the existing `allow_reverse` flag (`allow_reversing`).
> 3. MPPI is NOT removed — it stays as the experimental alternative.

## 1. Problem

The MPPI local layer makes one sampling optimizer responsible for path following, obstacle
avoidance, speed regulation, and direction choice at once. Live validation showed the cost:
weaving, near-goal misses, a reverse local-optimum lock-in, and gear thrash — each fixed by
another round of critic re-balancing. The classical split — global planner for the route, a
**pure tracking controller** for execution, with collision-regulated slowdown as the local
safety layer — makes each piece simple, deterministic, and debuggable alone. (Architecture
proposed by the user after observing MPPI's live behavior.)

Nav2's `RegulatedPurePursuitController` (RPP) is exactly that tracking controller: it
geometrically chases a lookahead point on the global path (already kinematically feasible from
`SmacPlannerHybrid`), regulates speed on curvature and obstacle proximity, supports reversing
on cusped paths (`allow_reversing`), and has an order of magnitude fewer tuning knobs than
MPPI. Verified available in the environment: RPP 1.1.20 with `allow_reversing` in the header.

## 2. Goal

Add `local_controller:=rpp` so the drive stack becomes: Smac global plan → **RPP tracks it
exactly** → `/cmd_vel` → acceleration bridge (+ GearArbiter) → gate. Everything already built
is reused; only the `controller_server` plugin/params change.

Non-goals: removing or retuning MPPI; `bridge`-mode changes; RPP fine-tuning beyond sane
defaults; new C++.

## 3. Architecture

```
local_controller:=mppi (default)          local_controller:=rpp
--------------------------------          --------------------------------
controller_server FollowPath =            controller_server FollowPath =
  nav2_mppi_controller::MPPIController      nav2_regulated_pure_pursuit_controller::
  (sampling planner+controller)              RegulatedPurePursuitController
                                             (pure tracking + collision-regulated speed)
```

Both run in the same `controller_server`, publish `/cmd_vel`, and share: Smac planner, goal
relay, cmd_vel bridge (acceleration + gear sequencing), recovery BTs, behavior_server,
lifecycle management, and the gate routing. Selection follows the proven `<let>`-selected
**file** pattern (never string `<param value>` overrides of typed params).

## 4. Components

### 4.1 New config — `config/nav2_rpp_controller.param.yaml`

`controller_server` params (same node name/structure as the MPPI file so the launch swap is a
file swap):
- Same `controller_frequency: 20.0`, progress checker, and **goal checker** (`xy 1.5`,
  `yaw 3.15`, stateful) as the MPPI file.
- `FollowPath.plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"`
  with Ackermann-appropriate starting values: `desired_linear_vel: 2.5`,
  `lookahead_dist: 4.0`, `use_velocity_scaled_lookahead_dist: true`,
  `min_lookahead_dist: 2.0`, `max_lookahead_dist: 6.0`, `lookahead_time: 1.5`,
  `use_rotate_to_heading: false` (Ackermann cannot rotate in place),
  `use_regulated_linear_velocity_scaling: true` (curvature slowdown),
  `use_cost_regulated_linear_velocity_scaling: true` (obstacle-proximity slowdown — the
  local safety layer), `regulated_linear_scaling_min_radius: 3.5` (matches the planner's
  minimum turning radius), `regulated_linear_scaling_min_speed: 0.25`,
  `approach_velocity_scaling_dist: 5.0` (decelerate into the goal),
  `min_approach_linear_velocity: 0.3`, `transform_tolerance: 0.1`,
  `max_robot_pose_search_dist: 10.0`, `allow_reversing: false` (overlay flips it).
- The **same 24×24 local costmap section** as the MPPI file (RPP's cost-regulated slowdown
  reads it).

### 4.2 RPP motion overlays — `config/nav2_rpp_forward.param.yaml` / `nav2_rpp_reverse.param.yaml`

Mirror of the MPPI overlays, but the reverse knob is `FollowPath.allow_reversing`
(`false` forward / `true` reverse). RPP detects cusps in the path and drives reverse segments
backwards natively; the bridge's GearArbiter turns the sign into DRIVE/REVERSE with
stop-and-shift exactly as with MPPI.

### 4.3 Launch — `nav2_offroad.launch.xml`

- New `<arg name="local_controller" default="mppi" description="mppi | rpp"/>`.
- `controller_param_file` cascade: MPPI file (default) → RPP file
  (`if local_controller==rpp`). The `controller_server` node's `<param from>` uses the var.
- The motion-overlay cascade becomes controller-aware (2-D: controller × reverse):
  `mppi_forward` (default) → `mppi_reverse` (if reverse_active) → `rpp_forward`
  (if rpp) → `rpp_reverse` (if rpp AND reverse_active). Later `<let>` wins, so ordering
  encodes the priority.
- Everything else (planner, relay, bridge, BTs, behavior_server, lifecycle list) untouched.

### 4.4 Docs

README args table + a short subsection: the split-pipeline rationale, the pairing
recommendation (`local_layer:=mppi_recovery local_controller:=rpp [allow_reverse:=true]`),
and that MPPI remains the experimental alternative.

## 5. Safety model

Unchanged from mppi/mppi_recovery: bridge stale-cmd_vel watchdog (now actively brakes), gate
filters, STOP-mode override, stop-and-shift gear sequencing. RPP adds deterministic
obstacle-proximity slowdown via cost-regulated scaling and decelerates into goals
(`approach_velocity_scaling`), replacing MPPI's ObstaclesCritic in this configuration.

## 6. Testing

- **Launch-parse:** the combination matrix parses — `local_controller` ∈ {mppi, rpp} ×
  `local_layer` ∈ {mppi, mppi_recovery} × `allow_reverse` ∈ {false, true}; plus `bridge`
  ignoring both args.
- **Regression:** default `local_controller:=mppi` selects the MPPI file and overlays —
  behavior identical to today.
- **Sim acceptance (same two goals as MPPI's):** with
  `local_layer:=mppi_recovery local_controller:=rpp allow_reverse:=true` — (A) 15 m forward
  goal: drives straight in, decelerates, REACHED, no weaving/orbiting; (B) 8 m behind goal:
  reverses (gear REVERSE observed) and REACHED. Verify `FollowPath.plugin` and
  `allow_reversing` live via `ros2 param get`.

## 7. Out of scope / follow-ups

- RPP tuning beyond the starting values (lookahead/speed polish after first drives).
- Removing MPPI or promoting RPP to default (decide after side-by-side validation).
- `bridge`-mode (full Autoware trajectory_follower split) — pending clean workspace.

## 8. Decisions (confirmed by the user, 2026-07-03)

1. `local_controller` arg (`mppi` default | `rpp`) on the mppi/mppi_recovery layers. ✔
2. RPP reverse tied to `allow_reverse` via `allow_reversing` overlays. ✔
3. MPPI retained as experimental alternative. ✔
