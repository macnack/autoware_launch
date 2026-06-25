# Off-road Trajectory Validator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deploy-blocking safety gate node that runs feasibility checks on the off-road trajectory and forwards it unchanged when feasible or emits a safe-stop trajectory when not, with diagnostics.

**Architecture:** A pure, ROS-free `TrajectoryValidatorCore` (fully unit-tested) holds all the checks. A thin `TrajectoryValidatorNode` digests `autoware_planning_msgs/Trajectory` + `nav_msgs/Odometry`, calls the core, and either republishes the input or publishes `TrajectoryBuilder::createStopTrajectory(...)`, updating a `diagnostic_updater` status.

**Tech Stack:** C++17, ROS 2 Humble (`rclcpp`), `autoware_planning_msgs`, `nav_msgs`, `diagnostic_updater`, `tf2`, GoogleTest (`ament_add_gtest`).

## Global Constraints

- Package: `autoware_nav2_offroad`, namespace `autoware::nav2_offroad`.
- Every source file starts with the package's Apache-2.0 header (copy the 14-line header verbatim from `src/mode_manager_core.cpp`).
- The core is ROS-free: it must compile with `g++ -std=c++17` against only the C++ standard library plus `mode_manager_core.hpp` (for `Pose2d`/`EgoState`). No ROS headers in `trajectory_validator_core.{hpp,cpp}` or its test.
- Continuity reuse: reuse `Pose2d`/`EgoState` and the continuity threshold semantics from `mode_manager_core.hpp`; implement the continuity math in the validator (do NOT instantiate `ModeManagerCore`).
- Checks are evaluated by category in this fixed order; the earliest failing category wins: `TOO_FEW_POINTS → NON_FINITE → VELOCITY → LONGITUDINAL_ACCEL → CURVATURE → LATERAL_ACCEL → CONTINUITY`.
- C++ standard: `CMAKE_CXX_STANDARD 17` (already set in `CMakeLists.txt`).

## Environment / how to build & test

This shell has **no ROS/colcon/gtest**; all builds run in docker. Define once per shell session:

```bash
export PKG=/home/maciej/autoware/src/launcher/autoware_launch/.claude/worktrees/feat-offroad-trajectory-validator/tier4_universe_launch/autoware_nav2_offroad
export IMG=macnack/autoware:omag-devel-cuda
```

**Fast core test loop** (used for every core task — compiles the committed gtest with the container's g++; ~5 s):

```bash
docker run --rm -v "$PKG":/pkg "$IMG" bash -lc \
  'g++ -std=c++17 -I/pkg/include /pkg/src/trajectory_validator_core.cpp /pkg/test/test_trajectory_validator_core.cpp -lgtest -lgtest_main -pthread -o /tmp/t && /tmp/t'
```

**Full integration** (used only in the final tasks — builds the sibling msgs pkg + this package and runs the ament gtest under colcon):

```bash
docker run --rm \
  -v "$PKG":/ws/src/autoware_nav2_offroad \
  -v "$PKG/../autoware_nav2_offroad_msgs":/ws/src/autoware_nav2_offroad_msgs \
  -w /ws "$IMG" bash -lc \
  'source /opt/autoware/setup.bash && colcon build --packages-up-to autoware_nav2_offroad && colcon test --packages-select autoware_nav2_offroad --event-handlers console_direct+ && colcon test-result --verbose'
```

All `git` commands run from `$PKG` (the worktree is on branch `feat/offroad-trajectory-validator`).

---

## File Structure

- Create `include/autoware_nav2_offroad/trajectory_validator_core.hpp` — types (`TrajPoint`, `FailedCheck`, `ValidatorParams`, `ValidationResult`), `toString(FailedCheck)`, `class TrajectoryValidatorCore`.
- Create `src/trajectory_validator_core.cpp` — `validate()` + `toString()` + anonymous-namespace helpers.
- Create `test/test_trajectory_validator_core.cpp` — gtest cases (one+ per check).
- Create `src/trajectory_validator_node.cpp` — the rclcpp wrapper.
- Create `config/trajectory_validator.param.yaml` — default thresholds.
- Modify `CMakeLists.txt` — core library, node executable, gtest registration, install.

---

## Task 1: Core skeleton + structural check

**Files:**
- Create: `include/autoware_nav2_offroad/trajectory_validator_core.hpp`
- Create: `src/trajectory_validator_core.cpp`
- Test: `test/test_trajectory_validator_core.cpp`

**Interfaces:**
- Consumes: `Pose2d`, `EgoState` from `autoware_nav2_offroad/mode_manager_core.hpp`.
- Produces: `struct TrajPoint{double x,y,yaw,velocity_mps,acceleration_mps2;}`; `enum class FailedCheck`; `struct ValidatorParams`; `struct ValidationResult`; `std::string toString(FailedCheck)`; `class TrajectoryValidatorCore` with `explicit TrajectoryValidatorCore(ValidatorParams)` and `ValidationResult validate(const std::vector<TrajPoint>&, const EgoState&) const`.

- [ ] **Step 1: Write the header.** Create `include/autoware_nav2_offroad/trajectory_validator_core.hpp` (with the Apache header) containing:

```cpp
#ifndef AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_
#define AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_

#include "autoware_nav2_offroad/mode_manager_core.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace autoware::nav2_offroad
{

// One trajectory sample, digested from autoware_planning_msgs::TrajectoryPoint.
struct TrajPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
};

enum class FailedCheck {
  NONE = 0,
  TOO_FEW_POINTS,
  NON_FINITE,
  VELOCITY,
  LONGITUDINAL_ACCEL,
  CURVATURE,
  LATERAL_ACCEL,
  CONTINUITY
};

std::string toString(FailedCheck check);

struct ValidatorParams
{
  std::size_t min_points{2};
  double max_velocity_mps{5.0};
  double max_longitudinal_accel_mps2{2.0};
  double max_lateral_accel_mps2{2.0};
  double max_curvature_1pm{1.0};
  // Continuity thresholds (same defaults as ModeManagerParams).
  double max_position_gap_m{2.0};
  double max_yaw_gap_rad{0.5};
  double max_velocity_step_mps{1.0};
};

struct ValidationResult
{
  bool feasible{true};
  FailedCheck check{FailedCheck::NONE};
  std::size_t point_index{0};
  double worst_value{0.0};
  double limit{0.0};
  std::string reason{};
};

/// Pure (ROS-free) feasibility gate for an off-road trajectory. Deterministic
/// and unit-testable: all inputs are digested plain data.
class TrajectoryValidatorCore
{
public:
  explicit TrajectoryValidatorCore(ValidatorParams params);

  ValidationResult validate(const std::vector<TrajPoint> & traj, const EgoState & ego) const;

private:
  ValidatorParams params_;
};

}  // namespace autoware::nav2_offroad

#endif  // AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_
```

- [ ] **Step 2: Write the failing test.** Create `test/test_trajectory_validator_core.cpp` (with the Apache header):

```cpp
#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::nav2_offroad::EgoState;
using autoware::nav2_offroad::FailedCheck;
using autoware::nav2_offroad::TrajPoint;
using autoware::nav2_offroad::TrajectoryValidatorCore;
using autoware::nav2_offroad::ValidationResult;
using autoware::nav2_offroad::ValidatorParams;

TrajPoint mk(double x, double y, double yaw, double v, double a = 0.0)
{
  return TrajPoint{x, y, yaw, v, a};
}

// A straight, slow, feasible trajectory of `n` points spaced 0.5 m along +x.
std::vector<TrajPoint> feasibleTraj(std::size_t n = 4, double v = 1.0)
{
  std::vector<TrajPoint> t;
  for (std::size_t i = 0; i < n; ++i) {
    t.push_back(mk(0.5 * static_cast<double>(i), 0.0, 0.0, v, 0.0));
  }
  return t;
}

// Ego sitting exactly on the first point (continuity satisfied).
EgoState egoAt(const TrajPoint & p)
{
  EgoState e;
  e.valid = true;
  e.pose.x = p.x;
  e.pose.y = p.y;
  e.pose.yaw = p.yaw;
  e.velocity_mps = p.velocity_mps;
  return e;
}

TEST(TrajectoryValidatorCore, FeasibleTrajectoryPasses)
{
  TrajectoryValidatorCore core{ValidatorParams{}};
  const auto traj = feasibleTraj();
  const ValidationResult r = core.validate(traj, egoAt(traj.front()));
  EXPECT_TRUE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::NONE);
}

TEST(TrajectoryValidatorCore, TooFewPointsFails)
{
  ValidatorParams p;
  p.min_points = 2;
  TrajectoryValidatorCore core{p};
  std::vector<TrajPoint> traj{mk(0.0, 0.0, 0.0, 0.0)};  // 1 point
  const ValidationResult r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::TOO_FEW_POINTS);
}
}  // namespace
```

- [ ] **Step 3: Run it to verify it fails.** Run the **Fast core test loop** command. Expected: compile/link error (`undefined reference to ...TrajectoryValidatorCore::validate...`), because the `.cpp` does not exist yet.

- [ ] **Step 4: Write the minimal implementation.** Create `src/trajectory_validator_core.cpp` (with the Apache header):

```cpp
#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace autoware::nav2_offroad
{
namespace
{
ValidationResult fail(
  FailedCheck check, std::size_t index, double value, double limit, std::string reason)
{
  ValidationResult r;
  r.feasible = false;
  r.check = check;
  r.point_index = index;
  r.worst_value = value;
  r.limit = limit;
  r.reason = std::move(reason);
  return r;
}
}  // namespace

std::string toString(FailedCheck check)
{
  switch (check) {
    case FailedCheck::NONE:
      return "none";
    case FailedCheck::TOO_FEW_POINTS:
      return "too_few_points";
    case FailedCheck::NON_FINITE:
      return "non_finite";
    case FailedCheck::VELOCITY:
      return "velocity";
    case FailedCheck::LONGITUDINAL_ACCEL:
      return "longitudinal_accel";
    case FailedCheck::CURVATURE:
      return "curvature";
    case FailedCheck::LATERAL_ACCEL:
      return "lateral_accel";
    case FailedCheck::CONTINUITY:
      return "continuity";
  }
  return "unknown";
}

TrajectoryValidatorCore::TrajectoryValidatorCore(ValidatorParams params) : params_(params) {}

ValidationResult TrajectoryValidatorCore::validate(
  const std::vector<TrajPoint> & traj, const EgoState & ego) const
{
  (void)ego;
  const ValidatorParams & p = params_;

  // 1. Structural.
  if (traj.size() < p.min_points) {
    return fail(
      FailedCheck::TOO_FEW_POINTS, 0, static_cast<double>(traj.size()),
      static_cast<double>(p.min_points),
      "trajectory has " + std::to_string(traj.size()) + " points, need at least " +
        std::to_string(p.min_points));
  }

  return ValidationResult{};  // feasible
}

}  // namespace autoware::nav2_offroad
```

- [ ] **Step 5: Run tests to verify they pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 2 tests.`

- [ ] **Step 6: Commit.**

```bash
cd "$PKG"
git add include/autoware_nav2_offroad/trajectory_validator_core.hpp src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): core skeleton + structural check"
```

---

## Task 2: Finite-value check

**Files:**
- Modify: `src/trajectory_validator_core.cpp`
- Test: `test/test_trajectory_validator_core.cpp`

**Interfaces:** unchanged (extends `validate` behavior).

- [ ] **Step 1: Write the failing tests.** Append inside the anonymous namespace in the test file:

```cpp
TEST(TrajectoryValidatorCore, NaNPositionFails)
{
  TrajectoryValidatorCore core{ValidatorParams{}};
  auto traj = feasibleTraj();
  traj[2].x = std::nan("");
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::NON_FINITE);
  EXPECT_EQ(r.point_index, 2u);
}

TEST(TrajectoryValidatorCore, InfVelocityFails)
{
  TrajectoryValidatorCore core{ValidatorParams{}};
  auto traj = feasibleTraj();
  traj[1].velocity_mps = std::numeric_limits<double>::infinity();
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::NON_FINITE);
}
```

Also add `#include <limits>` to the test file's includes.

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: the two new tests FAIL (result is feasible; NaN/Inf not yet detected).

- [ ] **Step 3: Implement.** In `src/trajectory_validator_core.cpp`, add this anonymous-namespace helper (next to `fail`):

```cpp
bool allFinite(const TrajPoint & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.yaw) &&
         std::isfinite(q.velocity_mps) && std::isfinite(q.acceleration_mps2);
}
```

Then in `validate`, immediately **after** the structural check block and **before** `return ValidationResult{};`, insert:

```cpp
  // 2. Finite.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    if (!allFinite(traj[i])) {
      return fail(
        FailedCheck::NON_FINITE, i, 0.0, 0.0,
        "non-finite value at point " + std::to_string(i));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 4 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): finite-value check"
```

---

## Task 3: Velocity limit check

**Files:** Modify `src/trajectory_validator_core.cpp`; Test `test/test_trajectory_validator_core.cpp`.

- [ ] **Step 1: Write the failing tests.** Append in the test namespace:

```cpp
TEST(TrajectoryValidatorCore, OverSpeedFails)
{
  ValidatorParams p;
  p.max_velocity_mps = 5.0;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj(4, 1.0);
  traj[3].velocity_mps = 6.0;  // over limit
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::VELOCITY);
  EXPECT_EQ(r.point_index, 3u);
  EXPECT_DOUBLE_EQ(r.worst_value, 6.0);
  EXPECT_DOUBLE_EQ(r.limit, 5.0);
}

TEST(TrajectoryValidatorCore, VelocityExactlyAtLimitPasses)
{
  ValidatorParams p;
  p.max_velocity_mps = 5.0;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj(4, 5.0);  // exactly at limit
  // Keep continuity happy: ego matches first point's 5.0 m/s.
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_TRUE(r.feasible);
}
```

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: `OverSpeedFails` FAILS (still feasible).

- [ ] **Step 3: Implement.** In `validate`, after the finite loop and before `return ValidationResult{};`, insert:

```cpp
  // 3. Velocity.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double v = std::fabs(traj[i].velocity_mps);
    if (v > p.max_velocity_mps) {
      return fail(
        FailedCheck::VELOCITY, i, v, p.max_velocity_mps,
        "speed " + std::to_string(v) + " m/s exceeds max " +
          std::to_string(p.max_velocity_mps) + " at point " + std::to_string(i));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 6 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): velocity limit check"
```

---

## Task 4: Longitudinal acceleration check

**Files:** Modify `src/trajectory_validator_core.cpp`; Test `test/test_trajectory_validator_core.cpp`.

- [ ] **Step 1: Write the failing test.** Append:

```cpp
TEST(TrajectoryValidatorCore, OverLongitudinalAccelFails)
{
  ValidatorParams p;
  p.max_longitudinal_accel_mps2 = 2.0;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj();
  traj[2].acceleration_mps2 = -3.5;  // |a| > 2.0
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::LONGITUDINAL_ACCEL);
  EXPECT_EQ(r.point_index, 2u);
  EXPECT_DOUBLE_EQ(r.worst_value, 3.5);
}
```

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: new test FAILS.

- [ ] **Step 3: Implement.** In `validate`, after the velocity loop and before `return ValidationResult{};`, insert:

```cpp
  // 4. Longitudinal acceleration.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double a = std::fabs(traj[i].acceleration_mps2);
    if (a > p.max_longitudinal_accel_mps2) {
      return fail(
        FailedCheck::LONGITUDINAL_ACCEL, i, a, p.max_longitudinal_accel_mps2,
        "|accel| " + std::to_string(a) + " m/s^2 exceeds max " +
          std::to_string(p.max_longitudinal_accel_mps2) + " at point " + std::to_string(i));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 7 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): longitudinal acceleration check"
```

---

## Task 5: Curvature check (with degenerate-geometry safety)

**Files:** Modify `src/trajectory_validator_core.cpp`; Test `test/test_trajectory_validator_core.cpp`.

- [ ] **Step 1: Write the failing tests.** Append:

```cpp
TEST(TrajectoryValidatorCore, OverCurvatureFails)
{
  ValidatorParams p;
  p.max_curvature_1pm = 1.0;  // min radius 1 m
  p.max_lateral_accel_mps2 = 1e9;  // disable lateral so curvature is the failure
  TrajectoryValidatorCore core{p};
  // Two points 0.5 m apart with a 1.0 rad heading change => kappa = 2.0 /m > 1.0.
  std::vector<TrajPoint> traj{mk(0.0, 0.0, 0.0, 0.5), mk(0.5, 0.0, 1.0, 0.5)};
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::CURVATURE);
  EXPECT_EQ(r.point_index, 0u);
}

TEST(TrajectoryValidatorCore, StackedPointsDoNotDivideByZero)
{
  ValidatorParams p;
  TrajectoryValidatorCore core{p};
  // Two coincident points (ds ~ 0) with a heading change: curvature is skipped.
  std::vector<TrajPoint> traj{mk(0.0, 0.0, 0.0, 0.0), mk(0.0, 0.0, 1.0, 0.0)};
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_TRUE(r.feasible);  // no divide-by-zero, no spurious curvature failure
}
```

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: `OverCurvatureFails` FAILS (still feasible).

- [ ] **Step 3: Implement.** Add this anonymous-namespace helper (next to `allFinite`):

```cpp
constexpr double kMinSegmentLength = 1e-6;  // m; avoids divide-by-zero on stacked points

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

// Arc length of segment i->i+1.
double segmentLength(const TrajPoint & a, const TrajPoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}
```

Then in `validate`, after the longitudinal loop and before `return ValidationResult{};`, insert:

```cpp
  // 5. Curvature (per segment; stacked points are skipped).
  for (std::size_t i = 0; i + 1 < traj.size(); ++i) {
    const double ds = segmentLength(traj[i], traj[i + 1]);
    if (ds <= kMinSegmentLength) {
      continue;
    }
    const double kappa = std::fabs(normalizeAngle(traj[i + 1].yaw - traj[i].yaw)) / ds;
    if (kappa > p.max_curvature_1pm) {
      return fail(
        FailedCheck::CURVATURE, i, kappa, p.max_curvature_1pm,
        "curvature " + std::to_string(kappa) + " /m exceeds max " +
          std::to_string(p.max_curvature_1pm) + " on segment " + std::to_string(i));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 9 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): curvature check with stacked-point guard"
```

---

## Task 6: Lateral acceleration check

**Files:** Modify `src/trajectory_validator_core.cpp`; Test `test/test_trajectory_validator_core.cpp`.

- [ ] **Step 1: Write the failing test.** Append:

```cpp
TEST(TrajectoryValidatorCore, OverLateralAccelFails)
{
  ValidatorParams p;
  p.max_curvature_1pm = 10.0;        // allow the curvature so lateral is the failure
  p.max_lateral_accel_mps2 = 2.0;
  p.max_velocity_mps = 100.0;
  TrajectoryValidatorCore core{p};
  // ds=0.5, dyaw=0.5 => kappa=1.0 /m; v=2 => lat = v^2*kappa = 4.0 > 2.0.
  std::vector<TrajPoint> traj{mk(0.0, 0.0, 0.0, 2.0), mk(0.5, 0.0, 0.5, 2.0)};
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::LATERAL_ACCEL);
  EXPECT_EQ(r.point_index, 0u);
  EXPECT_DOUBLE_EQ(r.worst_value, 4.0);
}
```

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: new test FAILS (curvature passes, lateral not yet checked).

- [ ] **Step 3: Implement.** In `validate`, add a **separate** loop after the curvature loop (keeps category ordering: all curvature before any lateral), before `return ValidationResult{};`:

```cpp
  // 6. Lateral acceleration (per segment; v^2 * kappa).
  for (std::size_t i = 0; i + 1 < traj.size(); ++i) {
    const double ds = segmentLength(traj[i], traj[i + 1]);
    if (ds <= kMinSegmentLength) {
      continue;
    }
    const double kappa = std::fabs(normalizeAngle(traj[i + 1].yaw - traj[i].yaw)) / ds;
    const double lat = traj[i].velocity_mps * traj[i].velocity_mps * kappa;
    if (lat > p.max_lateral_accel_mps2) {
      return fail(
        FailedCheck::LATERAL_ACCEL, i, lat, p.max_lateral_accel_mps2,
        "lateral accel " + std::to_string(lat) + " m/s^2 exceeds max " +
          std::to_string(p.max_lateral_accel_mps2) + " on segment " + std::to_string(i));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 10 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): lateral acceleration check"
```

---

## Task 7: Continuity-from-current-pose check

**Files:** Modify `src/trajectory_validator_core.cpp`; Test `test/test_trajectory_validator_core.cpp`.

- [ ] **Step 1: Write the failing tests.** Append:

```cpp
TEST(TrajectoryValidatorCore, PositionDiscontinuityFails)
{
  ValidatorParams p;
  p.max_position_gap_m = 2.0;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj();
  EgoState ego = egoAt(traj.front());
  ego.pose.x = 5.0;  // 5 m from first point
  const auto r = core.validate(traj, ego);
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::CONTINUITY);
}

TEST(TrajectoryValidatorCore, YawDiscontinuityWrapsAround)
{
  ValidatorParams p;
  p.max_yaw_gap_rad = 0.5;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj();
  EgoState ego = egoAt(traj.front());
  ego.pose.yaw = 3.0;  // ~3 rad from 0; wrapped gap is large
  const auto r = core.validate(traj, ego);
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::CONTINUITY);
}

TEST(TrajectoryValidatorCore, VelocityStepFails)
{
  ValidatorParams p;
  p.max_velocity_step_mps = 1.0;
  TrajectoryValidatorCore core{p};
  auto traj = feasibleTraj(4, 1.0);
  EgoState ego = egoAt(traj.front());
  ego.velocity_mps = 3.0;  // step of 2 m/s vs first point's 1 m/s
  const auto r = core.validate(traj, ego);
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::CONTINUITY);
}

TEST(TrajectoryValidatorCore, InvalidEgoSkipsContinuity)
{
  TrajectoryValidatorCore core{ValidatorParams{}};
  auto traj = feasibleTraj();
  EgoState ego;  // valid == false
  const auto r = core.validate(traj, ego);
  EXPECT_TRUE(r.feasible);  // continuity not evaluated without a valid ego
}
```

- [ ] **Step 2: Run to verify failure.** Run the **Fast core test loop**. Expected: the three discontinuity tests FAIL (still feasible).

- [ ] **Step 3: Implement.** In `validate`, after the lateral loop and before `return ValidationResult{};`, insert:

```cpp
  // 7. Continuity from current pose (only when ego is valid).
  if (ego.valid) {
    const TrajPoint & p0 = traj.front();
    const double pos_gap = std::hypot(ego.pose.x - p0.x, ego.pose.y - p0.y);
    if (pos_gap > p.max_position_gap_m) {
      return fail(
        FailedCheck::CONTINUITY, 0, pos_gap, p.max_position_gap_m,
        "position gap " + std::to_string(pos_gap) + " m exceeds max " +
          std::to_string(p.max_position_gap_m));
    }
    const double yaw_gap = std::fabs(normalizeAngle(ego.pose.yaw - p0.yaw));
    if (yaw_gap > p.max_yaw_gap_rad) {
      return fail(
        FailedCheck::CONTINUITY, 0, yaw_gap, p.max_yaw_gap_rad,
        "yaw gap " + std::to_string(yaw_gap) + " rad exceeds max " +
          std::to_string(p.max_yaw_gap_rad));
    }
    const double vel_gap = std::fabs(ego.velocity_mps - p0.velocity_mps);
    if (vel_gap > p.max_velocity_step_mps) {
      return fail(
        FailedCheck::CONTINUITY, 0, vel_gap, p.max_velocity_step_mps,
        "velocity step " + std::to_string(vel_gap) + " m/s exceeds max " +
          std::to_string(p.max_velocity_step_mps));
    }
  }
```

- [ ] **Step 4: Run to verify pass.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 14 tests.`

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add src/trajectory_validator_core.cpp test/test_trajectory_validator_core.cpp
git commit -m "feat(validator): continuity-from-current-pose check"
```

---

## Task 8: Check-ordering guarantee

**Files:** Test `test/test_trajectory_validator_core.cpp` (no implementation change expected — this pins the ordering contract).

- [ ] **Step 1: Write the test.** Append:

```cpp
TEST(TrajectoryValidatorCore, EarliestCategoryWins)
{
  ValidatorParams p;
  p.max_velocity_mps = 5.0;
  TrajectoryValidatorCore core{p};
  // Both a non-finite value (cat 2) and an over-speed (cat 3) are present;
  // NON_FINITE must win because it is the earlier category.
  std::vector<TrajPoint> traj{
    mk(0.0, 0.0, 0.0, 1.0), mk(0.5, 0.0, 0.0, 99.0), mk(1.0, 0.0, 0.0, std::nan(""))};
  const auto r = core.validate(traj, egoAt(traj.front()));
  EXPECT_FALSE(r.feasible);
  EXPECT_EQ(r.check, FailedCheck::NON_FINITE);
}
```

- [ ] **Step 2: Run to verify it passes.** Run the **Fast core test loop**. Expected: `[  PASSED  ] 15 tests.` (The ordering already holds from the implementation; this test locks it in. If it fails, the category loops are out of order — fix their order to match the Global Constraints.)

- [ ] **Step 3: Commit.**

```bash
cd "$PKG"
git add test/test_trajectory_validator_core.cpp
git commit -m "test(validator): pin check-ordering contract"
```

---

## Task 9: CMake wiring + colcon verification of the core

**Files:** Modify `CMakeLists.txt`.

**Interfaces:** Produces the CMake target `autoware_nav2_offroad_trajectory_validator_core` and the gtest `test_trajectory_validator_core`.

- [ ] **Step 1: Add the library.** In `CMakeLists.txt`, immediately after the `add_library(autoware_nav2_offroad_mode_manager_core ...)` block (and its `target_include_directories`), add:

```cmake
add_library(autoware_nav2_offroad_trajectory_validator_core
  src/trajectory_validator_core.cpp
)
target_include_directories(autoware_nav2_offroad_trajectory_validator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

- [ ] **Step 2: Install the library.** In the `install(TARGETS ...)` list, add the line `autoware_nav2_offroad_trajectory_validator_core` (alphabetical-ish, next to the other `..._core` targets).

- [ ] **Step 3: Register the gtest.** In the `if(BUILD_TESTING)` block, after the `test_mode_manager_core` block, add:

```cmake
  ament_add_gtest(test_trajectory_validator_core
    test/test_trajectory_validator_core.cpp
  )
  if(TARGET test_trajectory_validator_core)
    target_link_libraries(test_trajectory_validator_core
      autoware_nav2_offroad_trajectory_validator_core
    )
    target_include_directories(test_trajectory_validator_core PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
```

- [ ] **Step 4: Verify under colcon.** Run the **Full integration** command. Expected: build succeeds; `colcon test-result --verbose` shows `test_trajectory_validator_core` with `0 errors, 0 failures` and all 15 cases passing.

- [ ] **Step 5: Commit.**

```bash
cd "$PKG"
git add CMakeLists.txt
git commit -m "build(validator): wire core library + gtest registration"
```

---

## Task 10: Parameter file + validator node + final integration

**Files:**
- Create: `config/trajectory_validator.param.yaml`
- Create: `src/trajectory_validator_node.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `TrajectoryValidatorCore`, `ValidatorParams`, `ValidationResult`, `toString(FailedCheck)`; `TrajectoryBuilder`/`TrajectoryBuilderParams` from `trajectory_builder.hpp` (for `createStopTrajectory`).

- [ ] **Step 1: Create the param file.** Create `config/trajectory_validator.param.yaml`:

```yaml
/**:
  ros__parameters:
    min_points: 2
    max_velocity_mps: 5.0
    max_longitudinal_accel_mps2: 2.0
    max_lateral_accel_mps2: 2.0
    max_curvature_1pm: 1.0
    max_position_gap_m: 2.0
    max_yaw_gap_rad: 0.5
    max_velocity_step_mps: 1.0
```

- [ ] **Step 2: Create the node.** Create `src/trajectory_validator_node.cpp` (with the Apache header):

```cpp
#include "autoware_nav2_offroad/trajectory_builder.hpp"
#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::nav2_offroad
{
using autoware_planning_msgs::msg::Trajectory;

class TrajectoryValidatorNode : public rclcpp::Node
{
public:
  TrajectoryValidatorNode() : Node("trajectory_validator"), diagnostics_(this)
  {
    ValidatorParams vp;
    vp.min_points = static_cast<std::size_t>(
      std::max(static_cast<int>(declare_parameter<int>("min_points", 2)), 2));
    vp.max_velocity_mps = declare_parameter<double>("max_velocity_mps", 5.0);
    vp.max_longitudinal_accel_mps2 =
      declare_parameter<double>("max_longitudinal_accel_mps2", 2.0);
    vp.max_lateral_accel_mps2 = declare_parameter<double>("max_lateral_accel_mps2", 2.0);
    vp.max_curvature_1pm = declare_parameter<double>("max_curvature_1pm", 1.0);
    vp.max_position_gap_m = declare_parameter<double>("max_position_gap_m", 2.0);
    vp.max_yaw_gap_rad = declare_parameter<double>("max_yaw_gap_rad", 0.5);
    vp.max_velocity_step_mps = declare_parameter<double>("max_velocity_step_mps", 1.0);
    core_ = std::make_unique<TrajectoryValidatorCore>(vp);

    builder_ = std::make_unique<TrajectoryBuilder>(TrajectoryBuilderParams{});

    output_publisher_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});
    trajectory_subscription_ = create_subscription<Trajectory>(
      "~/input/trajectory", rclcpp::QoS{1},
      std::bind(&TrajectoryValidatorNode::onTrajectory, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      std::bind(&TrajectoryValidatorNode::onOdometry, this, std::placeholders::_1));

    diagnostics_.setHardwareID("trajectory_validator");
    diagnostics_.add("validation", this, &TrajectoryValidatorNode::produceDiagnostics);
  }

private:
  EgoState toEgoState() const
  {
    EgoState ego;
    if (!latest_odometry_) {
      return ego;
    }
    const auto & o = *latest_odometry_;
    if (
      !std::isfinite(o.pose.pose.position.x) || !std::isfinite(o.pose.pose.position.y) ||
      !std::isfinite(o.twist.twist.linear.x)) {
      return ego;  // invalid ego (non-finite) -> treated as no usable odometry
    }
    ego.valid = true;
    ego.pose.x = o.pose.pose.position.x;
    ego.pose.y = o.pose.pose.position.y;
    ego.pose.yaw = tf2::getYaw(o.pose.pose.orientation);
    ego.velocity_mps = o.twist.twist.linear.x;
    return ego;
  }

  static std::vector<TrajPoint> digest(const Trajectory & msg)
  {
    std::vector<TrajPoint> out;
    out.reserve(msg.points.size());
    for (const auto & pt : msg.points) {
      TrajPoint tp;
      tp.x = pt.pose.position.x;
      tp.y = pt.pose.position.y;
      tp.yaw = tf2::getYaw(pt.pose.orientation);
      tp.velocity_mps = pt.longitudinal_velocity_mps;
      tp.acceleration_mps2 = pt.acceleration_mps2;
      out.push_back(tp);
    }
    return out;
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odometry_ = *msg; }

  void onTrajectory(const Trajectory::SharedPtr msg)
  {
    const EgoState ego = toEgoState();
    if (!ego.valid) {
      awaiting_odometry_ = true;
      diagnostics_.force_update();
      return;  // no safe output without a current pose
    }
    awaiting_odometry_ = false;
    last_result_ = core_->validate(digest(*msg), ego);
    if (last_result_.feasible) {
      output_publisher_->publish(*msg);
    } else {
      output_publisher_->publish(
        builder_->createStopTrajectory(now(), latest_odometry_, std::nullopt));
    }
    diagnostics_.force_update();
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (awaiting_odometry_) {
      stat.summary(DiagnosticStatus::WARN, "awaiting odometry");
      return;
    }
    if (last_result_.feasible) {
      stat.summary(DiagnosticStatus::OK, "trajectory feasible");
    } else {
      stat.summary(DiagnosticStatus::ERROR, "safe stop: " + last_result_.reason);
    }
    stat.add("failed_check", toString(last_result_.check));
    stat.add("point_index", static_cast<int>(last_result_.point_index));
    stat.add("worst_value", last_result_.worst_value);
    stat.add("limit", last_result_.limit);
  }

  std::unique_ptr<TrajectoryValidatorCore> core_;
  std::unique_ptr<TrajectoryBuilder> builder_;
  diagnostic_updater::Updater diagnostics_;
  bool awaiting_odometry_{true};
  ValidationResult last_result_{};
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;

  rclcpp::Publisher<Trajectory>::SharedPtr output_publisher_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::TrajectoryValidatorNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Wire the node in CMake.** In `CMakeLists.txt`, after the `nav2_path_to_trajectory_bridge_node` executable block, add:

```cmake
add_executable(trajectory_validator_node
  src/trajectory_validator_node.cpp
)
target_link_libraries(trajectory_validator_node
  autoware_nav2_offroad_trajectory_validator_core
  autoware_nav2_offroad_trajectory_builder
)
target_include_directories(trajectory_validator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(trajectory_validator_node
  autoware_planning_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)
```

Then add `trajectory_validator_node` to the `install(TARGETS ...)` list.

- [ ] **Step 4: Confirm package.xml deps.** Verify `package.xml` already `<depend>`s on `autoware_planning_msgs`, `diagnostic_updater`, `geometry_msgs`, `nav_msgs`, `rclcpp`, `tf2`, `tf2_geometry_msgs`. Run:

```bash
grep -E 'autoware_planning_msgs|diagnostic_updater|geometry_msgs|nav_msgs|rclcpp|tf2|tf2_geometry_msgs' "$PKG/package.xml"
```

Expected: all present (they are used by existing nodes). If any is missing, add the corresponding `<depend>` line.

- [ ] **Step 5: Final integration build + test.** Run the **Full integration** command. Expected: `colcon build` succeeds (msgs pkg + package, including `trajectory_validator_node` linking and the gtest); `colcon test-result --verbose` shows `test_trajectory_validator_core` passing with 0 failures.

- [ ] **Step 6: Commit.**

```bash
cd "$PKG"
git add config/trajectory_validator.param.yaml src/trajectory_validator_node.cpp CMakeLists.txt
git commit -m "feat(validator): trajectory validator node + params + build wiring"
```

---

## Done criteria

- All 15 core gtest cases pass under both the fast g++ loop and colcon.
- `colcon build --packages-up-to autoware_nav2_offroad` succeeds with the node linked.
- The node forwards feasible trajectories unchanged and emits `createStopTrajectory(...)` on any failed check, with a `diagnostic_updater` status (OK / WARN-awaiting-odometry / ERROR+reason).
- No changes to the off-road planner or the mux/mode_manager.

## Notes / deferred

- **Node-level integration test** (rclcpp fixture asserting pass-through identity and safe-stop substitution) is deferred per the spec; the behavior is covered by core unit tests + the colcon link/build. Add later if the package grows node-test infra.
- **Launch wiring** (adding `trajectory_validator_node` to a launch file and remapping `~/input/trajectory` from the off-road planner output) is a separate integration step, intentionally out of scope for this safety-core item.
