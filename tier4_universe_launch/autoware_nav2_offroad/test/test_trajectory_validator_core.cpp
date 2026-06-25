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
