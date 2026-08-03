# MPPI drivable mode — closing the `local_layer:=mppi` control loop

**Status:** Design approved (user-confirmed 2026-07-02)
**Date:** 2026-07-02
**Package:** `autoware_nav2_offroad` (+ `tier4_control_launch` arg threading)
**Author:** Maciej Krupka

> **Confirmed decisions:**
> 1. **cmd_vel routing = vehicle_cmd_gate AUTO input** (via a new `auto_control_cmd_topic` launch arg), NOT the migration-ware `autoware_control_command_gate` and NOT the EXTERNAL/teleop path (ruled out by the control-docs review: type mismatch — see `BACKLOG.md` teach&repeat routing item).
> 2. **Base branch = `integration/nav2-offroad-on-main`** (PR #6), which already contains the MPPI bring-up (BACKLOG #11), the trajectory validator, and RTH.
> 3. **v1 is forward-only** (`vx_min: 0.0`); reverse driving (gear sequencing) is an explicit follow-up.

## 1. Problem

PR #6 ships `local_layer:=mppi`: `controller_server` (`nav2_mppi_controller`, Ackermann) + rolling
local costmap + `bt_navigator`, lifecycle-managed — but **bring-up only, not drivable**. Two seams
were explicitly deferred by the BACKLOG #11 design:
1. nothing translates the off-road goal (`/planning/offroad_goal`, PoseStamped topic) into the
   `NavigateToPose` **action** bt_navigator expects;
2. MPPI's `/cmd_vel` output reaches nothing — no route to `vehicle_cmd_gate`.

This spec closes both, making MPPI a true reactive local layer ("Option B"): Nav2 drives the
vehicle directly using a live local costmap, instead of the bridge→trajectory_follower path.

## 2. Architecture

```
/planning/offroad_goal (PoseStamped)          /planning/offroad_cancel (Bool)
        │                                             │
        ▼                                             ▼
offroad_goal_relay_node ── NavigateToPose action ──► bt_navigator ──► planner_server
        (NEW, thin action client)                         │  (ComputePathToPose)
                                                          ▼
                                            controller_server (nav2_mppi_controller)
                                              + rolling local_costmap (perception OGM)
                                                          │ /cmd_vel (Twist)
                                                          ▼
                                     cmd_vel_to_control_bridge (PORTED from teach&repeat:
                                       bicycle model + stale-cmd_vel hold-stop watchdog)
                                                          │ Control (+ GearCommand)
                                                          ▼
                    vehicle_cmd_gate input/auto/control_cmd  ◄── auto_control_cmd_topic arg
                    (AUTONOMOUS operation mode, standard engage)
```

- In `local_layer:=mppi`: the nav2_path_to_trajectory_bridge is suppressed (already the case);
  the trajectory_follower keeps running but publishes to its own (now unread) topic — **no
  dueling publishers, no new mux**.
- In `local_layer:=bridge` (default): `auto_control_cmd_topic` keeps its default
  `/control/trajectory_follower/control_cmd` — **zero behavior change**.

## 3. Components

### 3.1 `offroad_goal_relay_node` (new)
Thin action-client glue, modeled on the reviewed `nav2_navigate_through_poses_bridge`:
- Sub `/planning/offroad_goal` (PoseStamped) → send `nav2_msgs/action/NavigateToPose` goal.
- Sub `/planning/offroad_cancel` (`std_msgs/Bool`, true) → `async_cancel_all_goals`.
- Publishes `~/result` (UInt8 NONE/ACTIVE/SUCCEEDED/ABORTED/CANCELED, transient_local, startup
  NONE) + throttled feedback log; goal-rejection callback and `goal_in_flight_` re-entrancy
  guard ported from the NavThroughPoses bridge (both were review findings there — do not regress).
- Keeping the same goal/cancel topics means **RTH, the RViz Off-road Goal tool, and
  `offroad_demo_tour.py` work unchanged in mppi mode.**

### 3.2 `cmd_vel_to_control_bridge` (ported from `feat/teach-repeat-rth`)
Copy the current file state (not cherry-pick — the watchdog landed in a multi-file fix commit):
`include/autoware_nav2_offroad/cmd_vel_to_control.hpp`, `src/cmd_vel_to_control.cpp`,
`src/cmd_vel_to_control_bridge_node.cpp`, `test/test_cmd_vel_to_control.cpp`,
`config/cmd_vel_to_control.param.yaml`. Already reviewed + 5 gtests green there.
Extensions for mppi mode:
- `initial_enabled` param (default false; launch sets true in mppi mode) — the `~/enable`
  SetBool service stays for runtime gating.
- Gear output remapped to the existing `auto_gear_cmd_topic` (`/planning/gear_cmd` in offroad
  mode) instead of the external gear topic.
- Keep the stale-cmd_vel watchdog (hold-stop) — in mppi mode it is a primary safety element (§5).

### 3.3 Launch / arg threading
- `tier4_control_launch/control.launch.xml`: new `auto_control_cmd_topic` arg
  (default `/control/trajectory_follower/control_cmd`) used in the gate's
  `input/auto/control_cmd` remap — exactly mirroring the existing `auto_gear_cmd_topic`
  precedent from the off-road integration.
- Thread the arg through the control component the same way `auto_gear_cmd_topic` was threaded.
- `nav2_offroad.launch.xml` mppi branch additionally launches `offroad_goal_relay_node` and
  `cmd_vel_to_control_bridge` (initial_enabled:=true, wheelbase from `vehicle_wheelbase_m` arg),
  and sets `auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd` upstream (documented; actual
  override happens where the control component is included — autoware.launch.xml effective_* lets,
  following the navigation_mode pattern).
- Documented recommended pairing: `local_layer:=mppi occupancy_grid_source:=perception` so the
  rolling local costmap consumes the live AW perception OGM (the SLAM-global + perception-local
  dual-layer split stays a follow-up, BACKLOG #4).

## 4. v1 scope limits

- **Forward-only:** set `vx_min: 0.0` in `nav2_mppi_controller.param.yaml`. MPPI reversing needs
  gear-by-sign plus stop-before-gear-change sequencing in the bridge — deferred (same shape as
  the builder's backlog #6 reverse work). Note in TUNING.md.
- No lattice A/B changes; no BT customization (stock navigate_to_pose BT); no GPU tuning.

## 5. Safety model in mppi mode (explicit trade)

The trajectory validator does NOT apply (there is no Autoware trajectory to validate). The chain:
1. **Avoidance:** MPPI ObstaclesCritic on the live local costmap (reactive).
2. **Liveness:** bridge stale-cmd_vel watchdog → hold-stop Control if MPPI stops publishing.
3. **Final guard:** vehicle_cmd_gate filter (NOTE: offroad gate params are relaxed for sim —
   re-tighten before hardware, as documented in the real-world constraints review).
4. **Abort:** `/planning/offroad_cancel` → relay cancels NavigateToPose → MPPI stops → watchdog
   hold-stop. RViz "Activate ON-ROAD" button and RTH cancel_return reuse this unchanged.

## 6. Testing

- Ported bridge gtests (5) must stay green; relay no-server smoke (publish goal with no action
  server → `~/result` ABORTED after bounded wait, mirroring the NavThroughPoses bridge smoke).
- Launch-parse: `local_layer:=mppi` with the new nodes; `local_layer:=bridge` unchanged.
- End-to-end acceptance: **`scripts/offroad_demo_tour.py` under `local_layer:=mppi`** in the
  planning simulator — the tour must drive all legs (this also exercises goal→relay→BT→MPPI→
  bridge→gate).
- **Open sim-check item:** confirm the AUTONOMOUS engage transition succeeds without
  `/planning/trajectory` being published (operation_mode_transition_manager's engage checks
  reference the trajectory; `allow_autonomous_in_stopped: true` should make standstill engage
  pass — verify, else gate this mode's engage path explicitly).

## 7. Out of scope / follow-ups

- Reverse driving under MPPI (gear sequencing) — follow-up.
- `autoware_control_command_gate` migration (named command sources) — cleaner long-term
  arbitration once that gate is default.
- Dual-layer costmap (SLAM static global + perception local) — BACKLOG #4.
- Teach&repeat REMOTE routing — unaffected; this spec's AUTO-path routing may inform it.

## 8. Decisions (confirmed by the user, 2026-07-02)

1. AUTO-input routing (not control_command_gate). ✔
2. Base branch `integration/nav2-offroad-on-main`. ✔
3. Forward-only v1 (`vx_min: 0.0`); reverse is a follow-up. ✔
