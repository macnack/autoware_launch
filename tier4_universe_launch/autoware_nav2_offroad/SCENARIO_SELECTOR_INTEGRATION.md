# Design: OFFROAD as an `autoware_scenario_selector` scenario (Option C)

Status: **Draft for review** · Supersedes the standalone `trajectory_mode_manager`
once complete. See [MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md) §10 for context.

This folds off-road (Nav2) navigation into `autoware_scenario_selector` so it
becomes a first-class scenario alongside `LANEDRIVING` / `PARKING`, inheriting the
scenario state machine, the trajectory mux, diagnostics, and HMI/operation-mode
integration — and retiring the parallel mux/mode-manager.

## How `scenario_selector` works today (verified)

- Package: `src/universe/autoware_universe/planning/autoware_scenario_selector`
  (a composable `ScenarioSelectorNode`). **Different repo** from this package.
- Scenarios are string constants in `autoware_internal_planning_msgs/msg/Scenario`
  (`EMPTY`, `LANEDRIVING`, `PARKING`) — in the **core** `autoware_internal_msgs` repo.
- It is a **trajectory mux keyed on geometry**: subscribes per-scenario trajectory
  inputs (`input/lane_driving/trajectory`, `input/parking/trajectory`), the lanelet
  map, the route, odometry, and operation mode. `selectScenarioByPosition()` picks
  the scenario from ego/goal position vs the lanelet map (`isInLane`,
  `isInParkingLot`); `updateCurrentScenario()` adds hysteresis (5 s dwell to enter
  PARKING, 3 s to return). It republishes the active scenario's trajectory to
  `output/trajectory` and the scenario name to `output/scenario`.
- Wired in `tier4_planning_launch/.../scenario_planning.launch.xml`; the muxed
  trajectory feeds `velocity_smoother`.

## Three-repo change set

| Repo | Change |
|------|--------|
| `autoware_internal_msgs` (core) | add `string OFFROAD=OffRoad` to `Scenario.msg` |
| `autoware_universe` | add OFFROAD trajectory input, routing, and selection logic to `ScenarioSelectorNode` + its launch |
| `autoware_launch` | wire the Nav2 bridge trajectory into `scenario_planning.launch.xml` as the OFFROAD input; retire the standalone mux |

## Trigger-independent (mechanical) parts — ready to implement

These are unambiguous regardless of how OFFROAD is *selected*:

1. **Message:** `Scenario.msg` += `string OFFROAD=OffRoad`.
2. **Node routing:** add `offroad_trajectory_` + `onOffroadTrajectory()` +
   `input/offroad/trajectory` sub; handle `OFFROAD` in `getScenarioTrajectory()`
   so the Nav2 trajectory is republished to `output/trajectory` when active.
3. **Launch:** add `input_offroad_trajectory` to `scenario_selector.launch.xml`
   and route the Nav2 bridge output (`/nav2_offroad/planning/trajectory`) to it in
   `scenario_planning.launch.xml`; the bridge + Nav2 stack run as the OFFROAD planner.

## The pivotal decision: how is OFFROAD *selected*?

`scenario_selector` selects by lanelet geometry, but an off-road goal is **not** a
lanelet route, so OFFROAD selection cannot purely reuse the existing route logic.
Three options:

### Option 1 — Geometry only (ego off all lanelets)
OFFROAD when ego is outside every lanelet *and* outside parking lots (the natural
extension of "in parking lot → PARKING"). Pure, automatic, no new inputs.
- ✅ Fits the existing model exactly; no extra signals.
- ❌ Chicken-and-egg: you can only *enter* off-road by first driving off the road,
  so you can't start an off-road mission from on the road network.

### Option 2 — Off-road goal active (recommended)
`scenario_selector` subscribes to the off-road goal (`/planning/offroad_goal`).
OFFROAD is active while an off-road goal is set and unreached; returns to the
geometry-based choice when the goal is cleared/reached. Keeps our existing goal
flow; the goal *is* the intent signal.
- ✅ Lets you initiate off-road from anywhere; reuses the current goal topic.
- ✅ Minimal new surface (one goal subscription + an "active" latch).
- ❌ Slightly outside the pure-geometry model (but PARKING already special-cases).

### Option 3 — Explicit request input (AD-API / operator)
`scenario_selector` subscribes to an explicit off-road request (e.g. a Bool or an
operation-mode-style API), mirroring today's `change_mode`.
- ✅ Operator-authoritative; clean HMI story.
- ❌ Duplicates the trigger we are trying to fold in; least "automatic".

**Recommendation: Option 2** (off-road goal active), with the geometry check
(Option 1) as the fallback selection when no off-road goal is set. Hysteresis: enter
OFFROAD immediately on a fresh off-road goal; exit (to LANEDRIVING/PARKING by
position) after the goal is reached + stopped, with a short dwell like the existing
PARKING timeouts. Continuity guards from `MODE_MANAGER_DESIGN.md` §6 become the
scenario entry/exit conditions.

## Implementation order (once the trigger is confirmed)

1. Core msg: add `OFFROAD` constant (branch in `autoware_internal_msgs`).
2. Node (universe, TDD): trajectory routing for OFFROAD → selection logic for the
   chosen trigger → hysteresis. Extend the existing gtest interface test.
3. Launch (autoware_launch): wire the bridge as the OFFROAD planner input; retire
   the standalone `trajectory_mode_manager`/mux from `nav2_offroad.launch.xml`
   (keep the bridge, free map / relay, and Nav2 stack).
4. Migrate the RViz panel + `change_mode` to drive scenario selection (or retire in
   favor of the standard scenario HMI).

## Note on retiring the mode manager

The mode manager and its guards are not wasted: its continuity-guard logic
(`evaluateGuards`) and safe-stop behavior port directly into the scenario entry/exit
conditions. The `autoware_nav2_offroad_msgs` debug/status types can remain for
observability.
