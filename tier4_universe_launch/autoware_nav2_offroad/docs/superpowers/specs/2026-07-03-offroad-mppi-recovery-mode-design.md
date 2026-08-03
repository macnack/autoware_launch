# MPPI recovery mode — robust off-road navigation via the standard Nav2 loop

**Status:** Design approved (user-confirmed 2026-07-03)
**Date:** 2026-07-03
**Package:** `autoware_nav2_offroad`
**Author:** Maciej Krupka
**Branch / PR:** `feat/offroad-mppi-drive` / PR #7

> **Confirmed decisions (2026-07-03):**
> 1. New `local_layer` value **`mppi_recovery`**, parallel to `bridge` / `mppi`. The existing
>    `mppi` mode stays byte-for-byte unchanged.
> 2. **Forward-only** — no reverse. `DUBIN` motion model and `vx_min: 0.0` are retained.
> 3. **Forward-only-tailored recovery BT** — retry + ClearCostmap + wait + `drive_on_heading`.
>    **No `spin` / `backup`** (both are infeasible for a forward-only Ackermann vehicle).

## 1. Problem

The `mppi` mode (PR #7) makes `local_layer:=mppi` drivable, but to activate `bt_navigator`
without a `behavior_server` it uses a **recovery-free behavior tree**. That BT has no retry:
a single `SmacPlannerHybrid` failure aborts the whole `NavigateToPose` goal.

`SmacPlannerHybrid` is a forward-only (`DUBIN`) kinematic pose planner with a 3.5 m minimum
turning radius. When MPPI's path tracking weaves, the vehicle drifts laterally off the goal
line, and the planner cannot re-align to the exact goal **pose** within the remaining distance
→ it exhausts its approach iterations ("failed to create plan, exceeded maximum iterations")
→ `ComputePathToPose` fails → the recovery-free BT aborts the goal.

**Observed live (2026-07-03):** a ~22 m straight, heading-aligned goal — about as simple as it
gets — aborted ~6 m short of the target because a mid-drive replan failed and there was no
recovery to retry it.

**Root cause:** no recovery layer + brittle exact-pose planning. Standard Nav2 handles exactly
this with a `behavior_server`, a recovery BT (retry / clear-costmap / wait), and a looser
planner goal tolerance.

## 2. Goal

Add an **experimental** `local_layer:=mppi_recovery` that wraps the `mppi` drive stack in the
standard Nav2 robustness layer, so a transient planner failure **recovers instead of aborting**
— without touching the existing `mppi` mode.

Non-goals: changing `mppi` or `bridge` behavior; enabling reverse; retuning MPPI weaving
(tracked separately).

## 3. Architecture

Identical to `mppi` mode, plus a recovery layer:

```
/planning/offroad_goal → offroad_goal_relay → NavigateToPose → bt_navigator
     → planner_server (SmacPlannerHybrid)  + controller_server (nav2_mppi_controller)
       + global/local costmaps
     → /cmd_vel → cmd_vel_to_control_bridge → /nav2_offroad/mppi/control_cmd → vehicle_cmd_gate

  ADDED for mppi_recovery:
   + behavior_server (nav2_behaviors: wait, drive_on_heading) — lifecycle-managed
   + recovery BT: RecoveryNode wraps (ComputePathToPose → FollowPath) with
                  ClearEntireCostmap + Wait between retries
   + loosened Hybrid-planner goal tolerance + more approach iterations
```

## 4. Components

### 4.1 Launch — `nav2_offroad.launch.xml`

- Extend `local_layer` to accept `mppi_recovery`. Everywhere the mppi drive nodes are gated on
  `local_layer==mppi`, broaden the condition to **`mppi` OR `mppi_recovery`**: `controller_server`,
  `bt_navigator`, `offroad_goal_relay`, `cmd_vel_to_control_bridge`.
- `lifecycle_nodes` for `mppi_recovery` = `[planner_server, smoother_server, controller_server,
  bt_navigator, behavior_server]` (adds `behavior_server`).
- New `behavior_server` node, launched **only** for `mppi_recovery`.
- `bt_navigator` BT selection is mode-dependent:
  - `mppi` → the existing recovery-free BTs (`navigate_to_pose_no_recovery.xml` /
    `navigate_through_poses_no_recovery.xml`) — unchanged.
  - `mppi_recovery` → the new recovery BTs (§4.3).
- Planner goal-tolerance override applied for `mppi_recovery` (§4.4).

### 4.2 New config — `config/nav2_behavior_server.param.yaml`

`behavior_server` (nav2_behaviors) parameters:
- `behavior_plugins: ["wait", "drive_on_heading"]` — forward-only set; **no `spin`, no `backup`**.
- `local_costmap_topic: /local_costmap/costmap_raw`, `local_footprint_topic:
  /local_costmap/published_footprint` (the MPPI local costmap), `global_frame`, `robot_base_frame`,
  `cycle_frequency`, `transform_tolerance`, and the vehicle radius consistent with the costmaps.

### 4.3 Recovery BTs — `config/behavior_trees/`

`navigate_to_pose_recovery_offroad.xml`:
- `RecoveryNode(number_of_retries=6)` wrapping the drive tree
  `PipelineSequence( RateController(1 Hz){ ComputePathToPose planner_id="GridBased" }, FollowPath
  controller_id="FollowPath" )`.
- Recovery subtree (on failure): `Sequence( ClearEntireCostmap(global) → ClearEntireCostmap(local)
  → Wait(2.0 s) )`, then the RecoveryNode retries the drive tree. (Retry count and wait duration
  are the starting values; tunable.)
- **No `Spin` / `BackUp`** (forward-only). `drive_on_heading` is available in `behavior_server`
  for an optional forward nudge but is not required by the primary tree.
- A matching `navigate_through_poses_recovery_offroad.xml` (bt_navigator instantiates both
  navigators, so the through-poses default BT must also be recovery-server-compatible).

### 4.4 Planner tolerance override

For `mppi_recovery`, loosen `GridBased` so exact-pose parking is not demanded and give the
approach more room: `tolerance` ≈ 0.5 m (from 0.25) and a raised `max_on_approach_iterations`.
Delivered as a small overlay param file `config/nav2_offroad_recovery.param.yaml` layered over
`nav2_offroad.param.yaml` on `planner_server` (same pattern as the existing
`nav2_smac_lattice.param.yaml` overlay), applied only when `local_layer:=mppi_recovery`. The
overlay must not affect `mppi` / `bridge`.

### 4.5 Goal checker

Keep the controller `goal_checker` consistent with the planner tolerance (`xy_goal_tolerance`
≈ 0.5–1.0 m, a moderate `yaw_goal_tolerance`) so "planner can reach it" and "controller declares
reached" agree. Set in the recovery overlay, not in the shared `mppi` config.

## 5. Scope / v1 limits

- **Forward-only** — `vx_min: 0.0`, `DUBIN`. Reverse (REEDS_SHEPP + MPPI reverse + `backup`) is
  the natural next extension of this mode, explicitly out of scope now.
- **No `spin` / `backup` recoveries** — Ackermann-infeasible for a forward-only vehicle.
- `mppi` and `bridge` modes are untouched.
- Marked **EXPERIMENTAL** in the README until sim-validated.

## 6. Testing

- **Launch-parse:** `local_layer:=mppi_recovery` parses; `behavior_server` appears in the node
  set and in `lifecycle_nodes`.
- **Lifecycle:** `behavior_server` configures + activates; all five managed nodes reach `active`.
- **Regression:** `local_layer:=mppi` and `local_layer:=bridge` parse and behave unchanged
  (no `behavior_server`, recovery-free BT still selected for `mppi`).
- **Sim acceptance:** the ~22 m straight goal that aborted in `mppi` mode **reaches the goal**
  in `mppi_recovery` mode — i.e. a transient planner failure is recovered (retry + clear-costmap)
  rather than aborting the goal.

## 7. Out of scope / follow-ups

- Reverse driving (REEDS_SHEPP planner + MPPI reverse + `backup` recovery).
- MPPI weaving / tracking tuning (separate effort; reduces how often recovery is needed).
- Promoting `mppi_recovery` from experimental to the default MPPI mode once validated.

## 8. Decisions (confirmed by the user, 2026-07-03)

1. New `local_layer` value `mppi_recovery` (parallel to `bridge`/`mppi`). ✔
2. Forward-only; reverse deferred. ✔
3. Forward-only-tailored recovery BT — retry + ClearCostmap + wait + `drive_on_heading`,
   no `spin`/`backup`. ✔
