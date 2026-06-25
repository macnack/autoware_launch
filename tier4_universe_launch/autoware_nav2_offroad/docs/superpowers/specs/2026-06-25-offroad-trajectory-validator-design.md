# Off-road Trajectory Validator — Design

**Date:** 2026-06-25
**Backlog item:** #3 — off-road trajectory validator (DEPLOY BLOCKER)
**Package:** `autoware_nav2_offroad`
**Branch:** `feat/offroad-trajectory-validator` (pinned base `bbc52de4`)

## Purpose

Add a node that runs feasibility checks on the off-road trajectory before it
reaches the trajectory mux/controller. A trajectory that passes all checks is
forwarded unchanged (pass-through). A trajectory that fails any check is
replaced with a safe-stop trajectory, and the failure is reported via
diagnostics. This is a safety gate: the goal is that no infeasible off-road
trajectory (NaN/Inf, over-speed, over-curvature, discontinuous from the current
pose) ever reaches the controller.

## Scope

In scope:

- A pure, ROS-free validation core with full unit tests.
- A thin ROS node wrapping the core: subscribe, validate, pass-through or
  safe-stop, publish diagnostics.
- A parameter file with tunable thresholds.
- CMake/package wiring + registration in tests.

Out of scope (YAGNI for this item):

- Modifying the off-road planner or the mux/mode_manager.
- Smoothing or "repairing" an infeasible trajectory — we stop, we do not patch.
- On-road trajectory validation (the on-road stack has its own validators).

## Architecture

Two units, mirroring the existing `mode_manager_core` / `trajectory_mode_manager_node` split.

### Unit 1 — `TrajectoryValidatorCore` (pure, ROS-free)

Files: `include/autoware_nav2_offroad/trajectory_validator_core.hpp`,
`src/trajectory_validator_core.cpp`.

Reuses `Pose2d` and `EgoState` from `mode_manager_core.hpp` (no dependency on
the `ModeManagerCore` state machine itself — see "Reuse" below).

```cpp
struct TrajPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
};

enum class FailedCheck {
  NONE = 0, TOO_FEW_POINTS, NON_FINITE, VELOCITY, LONGITUDINAL_ACCEL,
  CURVATURE, LATERAL_ACCEL, CONTINUITY
};

struct ValidatorParams
{
  std::size_t min_points{2};
  double max_velocity_mps{5.0};
  double max_longitudinal_accel_mps2{2.0};
  double max_lateral_accel_mps2{2.0};
  double max_curvature_1pm{1.0};        // 1/min-turning-radius
  // Continuity thresholds (same defaults as ModeManagerParams).
  double max_position_gap_m{2.0};
  double max_yaw_gap_rad{0.5};
  double max_velocity_step_mps{1.0};
};

struct ValidationResult
{
  bool feasible{true};
  FailedCheck check{FailedCheck::NONE};
  std::size_t point_index{0};   // offending point, where applicable
  double worst_value{0.0};      // the measured value that tripped the check
  double limit{0.0};            // the threshold it exceeded
  std::string reason;           // human-readable, for diagnostics
};

class TrajectoryValidatorCore
{
public:
  explicit TrajectoryValidatorCore(ValidatorParams params);
  ValidationResult validate(const std::vector<TrajPoint> & traj,
                            const EgoState & ego) const;
private:
  ValidatorParams params_;
};
```

`validate` is `const`, deterministic, and takes only digested data — fully unit
testable with no ROS spin-up.

### Unit 2 — `TrajectoryValidatorNode` (ROS wrapper)

File: `src/trajectory_validator_node.cpp`.

- **Subscriptions**
  - `~/input/trajectory` — `autoware_planning_msgs/Trajectory` (the off-road
    stream; remapped from the off-road planner output in launch).
  - `/localization/kinematic_state` — `nav_msgs/Odometry` (ego pose + velocity).
- **Publications**
  - `~/output/trajectory` — `autoware_planning_msgs/Trajectory` (gated output).
  - Diagnostics via `diagnostic_updater` (status: OK when passing, ERROR with
    the failure reason when safe-stopping).
- **Per-message flow** (on each input trajectory):
  1. Digest the `Trajectory` msg into `std::vector<TrajPoint>` and the latest
     `Odometry` into `EgoState`.
  2. `result = core_.validate(traj, ego)`.
  3. If `result.feasible` → publish the **input msg unchanged** on
     `~/output/trajectory`.
  4. Else → publish `builder_->createStopTrajectory(now(), latest_odometry_,
     std::nullopt)` on `~/output/trajectory` (reusing the existing safe-stop
     primitive from `TrajectoryBuilder`).
  5. Update the diagnostic status from `result`.
- If no odometry has been received yet, the trajectory cannot be checked for
  continuity and the safe-stop itself needs odometry; in that case the node
  publishes nothing and reports a WARN diagnostic ("awaiting odometry").

## Data flow

```
off-road planner ──Trajectory──▶ trajectory_validator_node ──Trajectory──▶ mode/mux ──▶ controller
                                        │  ▲
                  /localization/kinematic_state (Odometry)
                                        │
                                        ▼
                                  diagnostics (OK / ERROR+reason)
```

The validator sits inline on the off-road stream, upstream of the mux's
`input/offroad`. Pass-through preserves the exact message (header, all point
fields); only on failure is the message substituted.

## The checks (evaluated in order; first failure wins → safe-stop)

Let `s_i` be the arc length between consecutive points (`hypot(dx, dy)`).

1. **Structural** — `traj.size() >= min_points`. Empty/too-short ⇒
   `TOO_FEW_POINTS`.
2. **Finite** — every `x, y, yaw, velocity_mps, acceleration_mps2` is finite.
   Any NaN/Inf ⇒ `NON_FINITE`.
3. **Velocity** — `|v_i| <= max_velocity_mps` for all `i` ⇒ else `VELOCITY`.
4. **Longitudinal accel** — `|a_i| <= max_longitudinal_accel_mps2` ⇒ else
   `LONGITUDINAL_ACCEL`. (Uses the point's `acceleration_mps2` field.)
5. **Curvature** — for each interior segment with `s_i > eps`,
   `κ_i = |normalizeAngle(yaw_{i+1} - yaw_i)| / s_i <= max_curvature_1pm` ⇒ else
   `CURVATURE`. Segments with `s_i <= eps` (stacked points) are skipped for
   curvature (they cannot define a turn radius).
6. **Lateral accel** — `v_i² · κ_i <= max_lateral_accel_mps2` ⇒ else
   `LATERAL_ACCEL`.
7. **Continuity from current pose** — gap between `ego` and `traj.front()`:
   - `position_gap = hypot(ego.x - p0.x, ego.y - p0.y) <= max_position_gap_m`
   - `yaw_gap = |normalizeAngle(ego.yaw - p0.yaw)| <= max_yaw_gap_rad`
   - `velocity_gap = |ego.v - p0.v| <= max_velocity_step_mps`
   All three must hold ⇒ else `CONTINUITY`. Only evaluated when `ego.valid`;
   if ego is invalid the node already short-circuits to WARN (see node flow).

`eps` for segment length is a small fixed constant (`1e-6 m`) to avoid division
by zero; it is not a tunable.

## Reuse / "build on evaluateGuards"

Decision: **reuse types + thresholds, own check** (option chosen during
brainstorming). The core `#include`s `mode_manager_core.hpp` to reuse `Pose2d`,
`EgoState`, and the same continuity threshold semantics
(`max_position_gap_m` / `max_yaw_gap_rad` / `max_velocity_step_mps`), and
implements the small continuity computation itself (identical math to
`ModeManagerCore::evaluateGuards` / its private `continuous`/`normalizeAngle`).
The validator does **not** instantiate `ModeManagerCore`, so it has no
dependency on the state-machine internals and stays independently testable.
A private `normalizeAngle` helper is duplicated in the validator core (one
small function; not worth a shared utility header for this item).

## Error handling

- **No odometry yet** → publish nothing, WARN diagnostic. Rationale: both the
  continuity check and the safe-stop need the current pose; without it we have
  no safe output to emit.
- **Infeasible trajectory** → safe-stop output, ERROR diagnostic with the
  offending check, point index, measured value, and limit.
- **Non-finite ego odometry** → treated as "no usable odometry" (WARN, no
  output), since a safe-stop built from NaN pose would itself be invalid.
- The core never throws; all failure modes are encoded in `ValidationResult`.

## Testing (TDD — tests written before implementation)

Pure-core gtest (`test/test_trajectory_validator_core.cpp`), one or more cases
per check:

- Passes a clean, feasible trajectory (all checks satisfied).
- Each check fails in isolation: too-few-points; NaN and Inf in each field;
  over-speed; over-longitudinal-accel; over-curvature; over-lateral-accel;
  each continuity gap (position, yaw with wraparound, velocity).
- Ordering: when multiple checks would fail, the earliest in the list is
  reported.
- Boundary values: exactly-at-limit passes; just-over fails.
- Degenerate geometry: stacked points (`s_i ≈ 0`) do not divide-by-zero and do
  not spuriously fail curvature.
- `ValidationResult` fields (`point_index`, `worst_value`, `limit`, `reason`)
  are populated correctly for a representative failure.

Node-level behavior (pass-through identity, safe-stop substitution, the
no-odometry WARN path) is covered by the core tests plus manual/integration
checks; a full rclcpp fixture is out of scope for this item unless the existing
package already has node-level test infrastructure to reuse.

## Files

New:

- `include/autoware_nav2_offroad/trajectory_validator_core.hpp`
- `src/trajectory_validator_core.cpp`
- `src/trajectory_validator_node.cpp`
- `test/test_trajectory_validator_core.cpp`
- `config/trajectory_validator.param.yaml`

Modified:

- `CMakeLists.txt` — add `autoware_nav2_offroad_trajectory_validator_core`
  library, the `trajectory_validator_node` executable (links the validator core
  + `trajectory_builder` for `createStopTrajectory`), install targets, and the
  `test_trajectory_validator_core` gtest.
- `package.xml` — already depends on `diagnostic_updater`,
  `autoware_planning_msgs`, `nav_msgs`, `rclcpp`, `tf2*`; add deps only if a
  gap is found during wiring.

## Default thresholds (in `trajectory_validator.param.yaml`)

| Param | Default | Rationale |
|---|---|---|
| `min_points` | 2 | need ≥2 to compute deltas |
| `max_velocity_mps` | 5.0 | off-road operates slowly |
| `max_longitudinal_accel_mps2` | 2.0 | comfort / traction limit |
| `max_lateral_accel_mps2` | 2.0 | rollover / traction margin |
| `max_curvature_1pm` | 1.0 | ~1 m minimum turning radius |
| `max_position_gap_m` | 2.0 | reused from `ModeManagerParams` |
| `max_yaw_gap_rad` | 0.5 | reused from `ModeManagerParams` |
| `max_velocity_step_mps` | 1.0 | reused from `ModeManagerParams` |

All thresholds are tunable; defaults are deliberately conservative for a
deploy-blocking safety gate.
