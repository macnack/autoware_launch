# Reverse driving for the off-road MPPI stack

**Status:** Design approved (user-confirmed 2026-07-03)
**Date:** 2026-07-03
**Package:** `autoware_nav2_offroad`
**Author:** Maciej Krupka
**Branch / PR:** `feat/offroad-mppi-drive` / PR #7

> **Confirmed decisions (2026-07-03):**
> 1. Exposed as **`allow_reverse:=true`**, a launch flag layered on `local_layer:=mppi_recovery`
>    (default `false` → forward-only, unchanged). Not a new mode.
> 2. Bridge gear change = **full stop-and-shift**: on a commanded direction flip, output zero
>    velocity and hold the current gear until `|vehicle_speed| < 0.1 m/s`, then shift gear, then
>    pass the new-direction command.
> 3. Starting values: MPPI `vx_min: -1.5` m/s; stop threshold `0.1` m/s.
> 4. `backup` recovery is folded into the same flag (enabled with reverse).

## 1. Problem

`autoware_nav2_offroad` is forward-only: `SmacPlannerHybrid` uses the `DUBIN` motion model
(forward arcs only) and the MPPI controller has `vx_min: 0.0`. A forward-only car cannot turn
around, so a goal placed **behind** the vehicle is unreachable — the planner can only produce
ever-widening forward loops and the vehicle drives away and never converges. This is the
confirmed root cause of "it can't reach goals I place anywhere" observed while validating
`mppi_recovery`.

Reverse driving (plan + control + gear handling) lets the vehicle maneuver to poses that are
not directly ahead.

## 2. Goal

Add an opt-in `allow_reverse:=true` flag on `local_layer:=mppi_recovery` that enables reverse
end-to-end: REEDS_SHEPP global planning, MPPI reverse sampling, safe gear sequencing in the
bridge, and a reverse-capable recovery behavior. Default (`false`) leaves `mppi_recovery`
forward-only and byte-for-byte unchanged.

Non-goals: the forward-goal tracking weave (separate MPPI-critic follow-up); reverse
speed/critic fine-tuning beyond the initial values; changing `mppi` or `bridge` modes.

## 3. Architecture

`allow_reverse:=true` (only meaningful with `local_layer:=mppi_recovery`) toggles four pieces,
each selected by a launch `<let>` pointing at an overlay param file or an alternate node
behavior — never via string-typed `<param value="$(var ...)">`, which cannot override a typed
double (the bug hit during `mppi_recovery` tuning):

```
allow_reverse=false (default)         allow_reverse=true
------------------------------        ------------------------------
planner  DUBIN                        planner  REEDS_SHEPP (fwd+rev arcs)
MPPI     vx_min 0.0                    MPPI     vx_min -1.5
bridge   gear = DRIVE (const)          bridge   gear state machine (stop-and-shift)
recovery no backup                     recovery backup enabled (behavior + BT)
```

## 4. Components

### 4.1 Launch — `nav2_offroad.launch.xml`

- New `<arg name="allow_reverse" default="false"/>`. It is only honored when
  `local_layer:=mppi_recovery` (documented; other modes ignore it).
- A `<let name="reverse_active" value="true"/>` computed as
  `local_layer==mppi_recovery AND allow_reverse==true`, used to select:
  - the planner overlay param file (§4.2),
  - the controller overlay param file (§4.3),
  - the bridge `enable_reverse` param (§4.4),
  - the recovery behavior config + BT (§4.5).
- Overlay param files are layered with a second `<param from="...">` on the relevant node,
  gated by splitting the node on `reverse_active` (node-level `if`/`unless`), the same pattern
  already used for `global_planner` hybrid/lattice — because Humble `launch_xml` forbids a
  conditional `<param from>` on a single node.

### 4.2 Planner overlay — `config/nav2_offroad_reverse.param.yaml`

Overlay on `planner_server` (layered over `nav2_offroad.param.yaml`) when `reverse_active`:
- `GridBased.motion_model_for_search: "REEDS_SHEPP"` (from `DUBIN`).
- `GridBased.reverse_penalty` kept (already `2.0`); tunable.

### 4.3 Controller overlay — `config/nav2_mppi_reverse.param.yaml`

Overlay on `controller_server` when `reverse_active`:
- `FollowPath.vx_min: -1.5` (from `0.0`). Reverse capped slower than forward `vx_max: 2.5`.

### 4.4 Bridge gear state machine — `cmd_vel_to_control_bridge_node` (+ `cmd_vel_to_control`)

New `enable_reverse` param (default `false`). When `false`, behavior is exactly as today
(`gear = DRIVE` constant). When `true`, a self-contained **GearArbiter** unit governs gear +
output velocity:

- **State:** `current_gear ∈ {DRIVE, REVERSE}` (init `DRIVE`).
- **Inputs per cycle:** `cmd_v` (`cmd_vel.linear.x`, signed) and `speed` (vehicle longitudinal
  speed from a new subscription to `/localization/kinematic_state`).
- **Deadband:** if `|cmd_v| < eps` (e.g. `0.05`), keep `current_gear`, output velocity 0.
- **Desired direction** `= sign(cmd_v)`. Gear direction: `DRIVE=+`, `REVERSE=-`.
- **Same direction** → output `cmd_v` unchanged, publish `current_gear`.
- **Opposite direction (flip requested):** output velocity `0.0` and **hold `current_gear**
  until `|speed| < stop_threshold` (0.1 m/s); then set `current_gear` to the desired direction;
  the next cycle passes the new-direction command through.
- Signed-velocity passthrough and the steering sign already handle reverse (existing code
  comment: "negative v naturally inverts the steer sign"); only gear selection + the stop-gate
  are new. The stale-`cmd_vel` watchdog (hold-stop) is unchanged.

GearArbiter is a pure unit (inputs → `{gear, out_velocity}`) with no ROS deps, so it is unit
tested directly.

### 4.5 Recovery — behavior_server + recovery BT (reverse variant)

When `reverse_active`:
- Load a separate `config/nav2_behavior_server_reverse.param.yaml` (selected by a `<let>` on
  `reverse_active`) whose `behavior_plugins` adds `backup`: `["wait", "drive_on_heading",
  "backup"]`. Forward-only keeps `nav2_behavior_server.param.yaml` (no backup).
- Select a reverse-capable recovery BT
  (`navigate_to_pose_recovery_reverse_offroad.xml` / through-poses variant): RecoveryNode with
  `ClearEntireCostmap → BackUp → Wait`. Forward-only keeps the no-backup BT.

## 5. Safety model additions

- **No gear slam:** the stop-and-shift gate guarantees the vehicle is nearly stopped before a
  DRIVE↔REVERSE change; hardware never sees a direction reversal while rolling.
- **Reverse is slower** (`vx_min -1.5` vs `vx_max 2.5`) — cautious by default.
- The bridge stale-`cmd_vel` hold-stop watchdog and the `vehicle_cmd_gate` filter still apply.
- Reverse remains **opt-in**; the default forward-only path is untouched.

## 6. Testing

- **Unit (gtest) — GearArbiter** (the core new logic): forward cmd in DRIVE passes through;
  reverse cmd while moving forward → output 0 + gear stays DRIVE until `speed<0.1`, then gear
  REVERSE and reverse cmd passes; deadband holds gear; symmetric for REVERSE→DRIVE. No gear
  change ever emitted while `|speed| ≥ 0.1`.
- **Launch-parse:** `allow_reverse:=true`/`false` with `local_layer:=mppi_recovery` parse;
  `allow_reverse` ignored for `mppi`/`bridge`.
- **Regression:** `enable_reverse:=false` bridge behavior identical to today (existing bridge
  gtests stay green; `gear = DRIVE`).
- **Sim acceptance:** with `allow_reverse:=true`, place a goal **behind** the vehicle and
  confirm it maneuvers (uses reverse) to reach it — the case forward-only cannot do.

## 7. Out of scope / follow-ups

- Forward-goal tracking weave (MPPI critic tuning) — separate.
- Reverse speed / REEDS_SHEPP penalty fine-tuning beyond the starting values.
- Clean-workspace validation of the true (non-bypass) drive path.

## 8. Decisions (confirmed by the user, 2026-07-03)

1. `allow_reverse` flag on `mppi_recovery` (not a new mode); default forward-only. ✔
2. Full stop-and-shift gear sequencing, `0.1 m/s` threshold. ✔
3. MPPI `vx_min: -1.5`. ✔
4. `backup` recovery folded into the flag. ✔
