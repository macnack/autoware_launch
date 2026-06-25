# Design — Nav2 MPPI controller as the off-road local layer (BACKLOG #11)

Date: 2026-06-25
Scope: **config + launch + docs** only. No sim-drive / GPU verification. No new C++.

## 1. Problem & context

The off-road stack runs **`planner_server` + `smoother_server` only**. The
`nav2_path_to_trajectory_bridge` calls `ComputePathToPose` + `SmoothPath`, converts the
result to an Autoware `Trajectory`, and Autoware's own `trajectory_follower` does the
control. There is **no Nav2 `controller_server` and no `bt_navigator`** in the stack.

Deep-research finding: the highest-value off-road change is adding the
**`nav2_mppi_controller`** as the local/control layer — the planner family DARPA RACER /
NASA JPL / Georgia Tech AutoRally use for rough-terrain kinodynamic driving. MPPI samples
control sequences forward through a dynamics model (feasible by construction), handles
traction/slope/contact dynamics + reactive avoidance, reverses by default (`vx_min` ≈
−0.35), and is cost-shapeable. `SmacPlannerLattice` is at most an optional A/B test.

## 2. Goals / non-goals

Goals: add `nav2_mppi_controller` (Ackermann) as a **selectable, opt-in** local layer
wired into the existing launch; keep the bridge + `trajectory_follower` path the
**default**; add the `SmacPlannerLattice` A/B toggle (Hybrid-A\* default); document.

Non-goals (deferred): routing MPPI `/cmd_vel` → Autoware `Control` → `vehicle_cmd_gate`
(shared open seam with teach&repeat); a goal→NavigateToPose relay; sim-drive / GPU /
critic tuning; traversability / dynamic-obstacle costmap layers (BACKLOG #4).

## 3. Architecture

MPPI cannot act alone: to drive the global path it must receive that path and emit
`/cmd_vel`. In Nav2 the orchestration connecting planner_server → controller_server is
`bt_navigator` (NavigateToPose BT runs ComputePathToPose then FollowPath). So MPPI
bring-up = **controller_server (nav2_mppi_controller) + local_costmap + bt_navigator**,
all lifecycle-managed. In `mppi` mode the bridge is not launched. MPPI produces `/cmd_vel`;
routing it to the gate is out of scope, so `mppi` mode is bring-up only, not yet drivable.

## 4. Launch interface (`nav2_offroad.launch.xml`)

- `local_layer` = `bridge` (default) | `mppi`. `mppi` runs controller_server + bt_navigator,
  adds them to lifecycle_manager node_names, loads the MPPI param file, suppresses the bridge.
- `global_planner` = `smac_hybrid` (default) | `lattice`. `lattice` overlays SmacPlannerLattice
  on top of nav2_offroad.param.yaml (which still supplies global_costmap).

## 5. Config files (new; loaded only when selected)

- `config/nav2_mppi_controller.param.yaml` — controller_server (MPPIController, Ackermann,
  min_turning_r 3.5, vx_max 2.0, vx_min -0.35, critics incl. ObstaclesCritic) + rolling
  local_costmap (static + inflation from /nav2_offroad/costmap/occupancy_grid).
- `config/nav2_bt_navigator.param.yaml` — bt_navigator with stock navigate_to_pose BT.
- `config/nav2_smac_lattice.param.yaml` — SmacPlannerLattice overlay (frozen-radius caveat).

## 6. Docs

BACKLOG #11 (implemented vs remaining), README (args + nodes), TUNING (MPPI pointer),
this spec.

## 7. Testing / verification

No new C++ ⇒ no unit. Verify: YAML well-formedness, launch/package XML parse, default
`bridge` path unchanged. No MPPI driving behavior verified (no GPU/sim-drive).

## 8. Risks

GPU needed for MPPI rollouts; local-minima failure mode; cmd_vel→gate seam blocks drive
until separately resolved; critics are untuned Nav2 defaults adapted to Ackermann.
