# Design: `trajectory_mode_manager`

Status: **Draft for review** · Owner: Maciej Krupka · Supersedes: `trajectory_mode_mux`

This document specifies a safety-aware replacement for the current
`trajectory_mode_mux` so that switching between the Autoware on-road planner and
the Nav2 off-road planner is viable on a **real vehicle** and supports dynamic
runtime configuration.

It is a design artifact only — no code is implied as written until the open
decisions in [§11](#11-decisions-to-confirm) are resolved.

---

## 1. Problem statement

`trajectory_mode_mux` is a stateless `std_srvs/SetBool` switch: it forwards one
of two trajectories to `/planning/trajectory` based on a boolean. That is
adequate for bench/sim but unsafe on a vehicle because it has:

- no validation that the target planner is healthy and producing a *valid*
  trajectory **before** cutover;
- no velocity/heading continuity guarantee at the splice point (a step change in
  commanded velocity is passed straight to the controller);
- no fault handling (if the active source goes silent, the controller starves);
- no integration with diagnostics, operation mode, HMI, or MRM;
- both planners running hot regardless of which is active (ECU compute waste).

## 2. Goals / non-goals

**Goals**

- Authoritative, observable **mode state machine** with guarded transitions.
- **No publication gap** on `/planning/trajectory` during a switch.
- **Validated, continuous** handover (reject or ramp discontinuities).
- Orchestrate **Nav2 lifecycle** so the inactive planner can be deactivated.
- First-class **diagnostics** + a clean **service/topic interface** for HMI / AD API.
- Deterministic **fault behavior** (safe stop + error escalation).

**Non-goals (v1)**

- Automatic map-containment/geofence switching (designed-for, not implemented).
- Replacing `autoware_scenario_selector` (that is the eventual migration target,
  see [§10](#10-migration-to-scenario_selector-option-c)).
- Blending two trajectories into one (we commit to one source; we only *ramp
  velocity* at the splice, we do not fuse geometry).

## 3. Architecture overview

```
 on-road planner ─ /planning/trajectory_pre_mux ─┐
                                                  │
 nav2 bridge ───── /nav2_offroad/.../trajectory ─┤   ┌─────────────────────────┐
                                                  ├──►│  trajectory_mode_manager │──► /planning/trajectory
 /localization/kinematic_state ───────────────────┤   │  • state machine        │
                                                  │   │  • transition guards     │──► ~/status (latched)
 nav2_lifecycle_manager (manage_nodes) ◄──────────┘   │  • nav2 lifecycle ctrl   │──► /diagnostics
        ▲                                             │  • safe-stop fallback    │
        └──── activate / deactivate nav2 servers ─────└─────────────────────────┘
                                                          ▲
                            ~/change_mode (service)  ─────┘   (HMI / AD API / operator)
```

The manager **owns** `/planning/trajectory`. It is the only node permitted to
publish it (the `tier4_planning_launch` patch already routes the validator to
`/planning/trajectory_pre_mux` to make this possible).

## 4. State machine

```
                 request NAV2            target valid + continuous
   ┌────────────┐ ───────────► ┌──────────────────┐ ──────────► ┌──────────────┐
   │ AW_PLANNING│               │ TRANSITION_TO_NAV2│             │ NAV2_OFFROAD │
   │ (route AW) │ ◄─────────── └──────────────────┘ ◄────────── │ (route nav2) │
   └────────────┘  request AW    ▲       │ abort/timeout          └──────────────┘
        ▲   │                    │       ▼                              │   ▲
        │   └───────────► ┌──────────────────┐ ◄────────────────────────┘   │
        │   request AW    │ TRANSITION_TO_AW │  target valid + continuous    │
        │                 └──────────────────┘ ──────────────────────────────┘
        │                         │ abort/timeout
        │                         ▼
        │                  ┌──────────────┐
        └───── recover ────│  SAFE_STOP   │  (publish jerk-limited decel-to-zero,
                           └──────────────┘   raise ERROR diag)
```

Additional startup state **`STANDBY`**: before localization/odometry is
available the manager holds `SAFE_STOP`-style output and refuses transitions.

State definitions:

| State | Routes to output | nav2 lifecycle | Diag level |
|-------|------------------|----------------|------------|
| `STANDBY` | safe-stop trajectory | inactive | WARN |
| `AW_PLANNING` | on-road trajectory | inactive (on-demand) / active (hot) | OK |
| `TRANSITION_TO_NAV2` | **still on-road** until commit | activating | WARN |
| `NAV2_OFFROAD` | off-road trajectory | active | OK |
| `TRANSITION_TO_AW` | **still off-road** until commit | active until commit | WARN |
| `SAFE_STOP` | safe-stop trajectory | unchanged | ERROR |

Key rule: **the source is only switched at the commit instant**, after guards
pass. The previously-active source keeps flowing during the transition, so the
controller never sees a gap or an unvalidated trajectory.

## 5. Transition sequence (AW → NAV2)

1. `~/change_mode{NAV2_OFFROAD}` received → enter `TRANSITION_TO_NAV2`.
2. Ask `nav2_lifecycle_manager` to `startup`/`resume` (if on-demand). Keep
   routing the on-road trajectory.
3. Wait for an off-road trajectory that is **fresh** (age < `target_trajectory_timeout_s`)
   and **valid** (see [§6](#6-transition-guards)).
4. Check **continuity** against current ego state from `/localization/kinematic_state`.
5. If all guards pass within the timeout → **commit**: switch routing to off-road,
   transition to `NAV2_OFFROAD`. Optionally ramp commanded velocity over
   `velocity_ramp_time_s` to avoid a step.
6. If on-demand and configured, leave AW planner running (it is cheap relative to
   perception) but stop *routing* it.
7. On timeout / invalid / discontinuity beyond threshold → **abort**: if the
   on-road source is still healthy, fall back to `AW_PLANNING`; else `SAFE_STOP`.

NAV2 → AW is symmetric; deactivating nav2 (on-demand) happens after commit.

## 6. Transition guards (the safety core)

A transition commits only if **all** hold:

| Guard | Check | Param |
|-------|-------|-------|
| Freshness | target trajectory stamp age < limit | `target_trajectory_timeout_s` (1.0) |
| Validity | finite values, ≥ N points, point spacing, curvature within limit — reuse `planning_validator` if `use_planning_validator`, else internal checks | `min_valid_points` (3), `max_curvature` |
| Position continuity | dist(ego, target.first) ≤ limit | `max_position_gap_m` (2.0) |
| Heading continuity | |yaw(ego) − yaw(target.first)| ≤ limit | `max_yaw_gap_rad` (0.5) |
| Velocity continuity | |v(ego) − v(target.first)| ≤ limit, OR ramp over time | `max_velocity_step_mps` (1.0), `velocity_ramp_time_s` |
| Source liveness | previous source still fresh until commit | reuses freshness |

No-gap invariant: the output publish timer runs at a fixed rate independent of
input arrival; it always emits *something* (current routed source, ramped, or
safe-stop). `vehicle_cmd_gate`'s timeout watchdog must never fire because of a
transition.

## 7. Nav2 lifecycle orchestration

Nav2 servers (`planner_server`, `smoother_server`) are `LifecycleNode`s managed
by `nav2_lifecycle_manager`. The manager controls them via the lifecycle
manager's `manage_nodes` service (`STARTUP` / `PAUSE` / `RESUME` / `SHUTDOWN`)
rather than driving each node directly.

- `planner_lifecycle: on_demand` (default) — nav2 is `PAUSE`d in `AW_PLANNING`
  and `RESUME`d when transitioning to off-road. Saves ECU compute.
- `planner_lifecycle: hot` — nav2 stays active always; transitions only reroute.

## 8. ROS interface

**Node:** `trajectory_mode_manager` (executable replaces `trajectory_mode_mux_node`).

Services (the "service server"):

| Service | Type | Purpose |
|---------|------|---------|
| `~/change_mode` | `autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode` — req `{string target_mode, bool force}`, resp `{bool accepted, string current_mode, string message}` | request a mode |
| `~/set_mode` | `std_srvs/srv/SetBool` | **deprecated** compat shim: `true`→NAV2_OFFROAD, `false`→AW_PLANNING |

Topics:

| Dir | Topic | Type | Notes |
|-----|-------|------|-------|
| pub | `output/trajectory` → `/planning/trajectory` | `autoware_planning_msgs/Trajectory` | fixed-rate, never gaps |
| pub | `~/status` | `autoware_nav2_offroad_msgs/msg/TrajectoryModeState`, transient_local | current/requested mode, transition, active_planner, fault_reason, stamp |
| pub | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | via `diagnostic_updater` |
| sub | `input/onroad/trajectory` | `autoware_planning_msgs/Trajectory` | from `/planning/trajectory_pre_mux` |
| sub | `input/offroad/trajectory` | `autoware_planning_msgs/Trajectory` | from nav2 bridge |
| sub | `/localization/kinematic_state` | `nav_msgs/Odometry` | continuity checks + safe-stop base pose |
| client | `<lifecycle_manager>/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | nav2 on-demand control |

Parameters (initial set):

| Param | Default | Meaning |
|-------|---------|---------|
| `mode_on_startup` | `AW_PLANNING` | initial mode after STANDBY clears |
| `transition_policy` | `manual` | `manual` \| `map_containment` \| `geofence` (only `manual` in v1) |
| `planner_lifecycle` | `on_demand` | `on_demand` \| `hot` |
| `publish_rate_hz` | `10.0` | output rate (match planning) |
| `target_trajectory_timeout_s` | `1.0` | freshness limit |
| `max_position_gap_m` | `2.0` | continuity |
| `max_yaw_gap_rad` | `0.5` | continuity |
| `max_velocity_step_mps` | `1.0` | continuity |
| `velocity_ramp_time_s` | `1.0` | ramp instead of step |
| `use_planning_validator` | `true` | validate target via validator vs internal |
| `safe_stop_decel_mps2` | `-1.5` | safe-stop profile |
| `lifecycle_manager_service` | `/lifecycle_manager_navigation/manage_nodes` | nav2 control |

## 9. Diagnostics, AD API, and MRM

- **Diagnostics:** publish a `mode_manager` diag — OK in steady states, WARN in
  transition/standby, ERROR in `SAFE_STOP`. Add it as a monitored node in the
  `*-nav2-offroad.yaml` diagnostic graphs so `diagnostic_graph_aggregator` and
  `system_error_monitor` see it.
- **MRM:** on `SAFE_STOP`, publish a jerk-limited decel-to-zero (reuse
  `TrajectoryBuilder::createStopTrajectory`) **and** raise ERROR so the existing
  `mrm_handler` / `vehicle_cmd_gate` emergency path can escalate. Never stop
  publishing the output topic.
- **AD API / HMI (integration point):** map modes onto an operation-mode-style
  AD API or a dedicated planner-selection API so RViz/HMI drive `~/change_mode`
  instead of a raw CLI call. Detailed in the Option C migration.

## 10. Migration to `scenario_selector` (Option C) — agreed target

This is the **agreed end-state**. The standalone `trajectory_mode_manager` is the
stepping stone; OFFROAD then becomes a first-class scenario inside
`autoware_scenario_selector` alongside `LANEDRIVING` / `PARKING`.

The state machine, guards, and lifecycle control defined here are intentionally
the same primitives the scenario selector already uses, so the port is mostly
relocation of logic rather than redesign. Concretely:

1. Add an `OFFROAD` scenario to `autoware_scenario_selector` (enum + transition
   rules), reusing the guards from [§6](#6-transition-guards) as the scenario's
   entry/exit conditions.
2. Move trajectory routing into the selector's existing output path (it already
   arbitrates scenario trajectories), retiring this node's mux role.
3. Drive Nav2 lifecycle ([§7](#7-nav2-lifecycle-orchestration)) from the
   scenario's activate/deactivate hooks.
4. Replace the `~/change_mode` service with the selector's existing
   scenario/operation-mode interface, inheriting **diagnostics, MRM, and HMI/AD
   API** integration for free.
5. Enable **automatic** transitions (map-containment / geofence), which are the
   selector's native model (it already auto-selects `PARKING` vs `LANEDRIVING`).

Keeping the interface (`~/status`, the `autoware_nav2_offroad_msgs` types, guard
params) stable through v1 makes this migration mechanical.

## 11. Decisions (confirmed)

1. **Transition trigger — `manual`** ✅ (operator/HMI via `~/change_mode`).
   Automatic `map_containment` is deferred to the `scenario_selector` migration
   ([§10](#10-migration-to-scenario_selector-option-c-agreed-target)), which is
   its native model.
2. **Nav2 role — off-road-only, operator-initiated** ✅. Not a health-triggered
   automatic fallback for the on-road planner (that would need stricter
   auto-switch + arbitration and is out of scope).
3. **Planner lifecycle — `on_demand`** ✅ (deactivate Nav2 in `AW_PLANNING` to
   save ECU compute). `hot` remains available as a parameter if instant
   transitions are later required.

## 12. Test plan

- **Unit (gtest):** state-machine transitions incl. abort/timeout; each guard in
  isolation; safe-stop generation. (Extends the existing
  `test_trajectory_mode_mux` suite.)
- **Integration (launch_test):** spin manager + fake on-road/off-road publishers
  + fake odometry; assert no output gap across a switch, abort on stale target,
  SAFE_STOP on dual-source loss, correct nav2 `manage_nodes` calls.
- **Sim:** planning_simulator, exercise switches while moving; inspect velocity
  continuity in rosbag.
- **Vehicle:** closed course, low speed, e-stop ready; log all transitions.

## 13. Implementation notes

- **Interface package — decided** ✅: the IDL lives in a separate
  `autoware_nav2_offroad_msgs` package (built and lint-clean), holding
  `srv/ChangeTrajectoryMode` and `msg/TrajectoryModeState`. This keeps the
  `rosidl` build deps out of the runtime node package and lets HMI/AD-API/tests
  depend on the interface alone.
- Single-threaded executor with a mutex-guarded state struct and a fixed-rate
  output timer is sufficient; transitions are evaluated in the timer + callbacks.
