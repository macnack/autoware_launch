// Copyright 2026 Maciej Krupka maciej.krupka@put.poznan.pl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

TEST(TrajectoryValidatorCore, OverCurvatureFails)
{
  ValidatorParams p;
  p.max_curvature_1pm = 1.0;       // min radius 1 m
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

TEST(TrajectoryValidatorCore, OverLateralAccelFails)
{
  ValidatorParams p;
  p.max_curvature_1pm = 10.0;  // allow the curvature so lateral is the failure
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
}  // namespace
