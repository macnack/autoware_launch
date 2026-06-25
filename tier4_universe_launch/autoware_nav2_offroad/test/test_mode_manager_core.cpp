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

#include "autoware_nav2_offroad/mode_manager_core.hpp"

#include <gtest/gtest.h>

#include <string>

namespace
{
using autoware::nav2_offroad::Decision;
using autoware::nav2_offroad::EgoState;
using autoware::nav2_offroad::Mode;
using autoware::nav2_offroad::ModeManagerCore;
using autoware::nav2_offroad::ModeManagerParams;
using autoware::nav2_offroad::Route;
using autoware::nav2_offroad::SourceState;
using autoware::nav2_offroad::Transition;

SourceState freshValid(const double x = 0.0, const double y = 0.0, const double v = 0.0)
{
  SourceState s;
  s.present = true;
  s.age_s = 0.0;
  s.valid = true;
  s.first_pose = {x, y, 0.0};
  s.first_velocity_mps = v;
  return s;
}

SourceState absent()
{
  return SourceState{};
}

SourceState staleSource()
{
  SourceState s;
  s.present = true;
  s.age_s = 2.0;  // older than the default 1.0s timeout
  s.valid = true;
  return s;
}

EgoState egoAt(const double x = 0.0, const double y = 0.0, const double v = 0.0)
{
  EgoState e;
  e.valid = true;
  e.pose = {x, y, 0.0};
  e.velocity_mps = v;
  return e;
}

EgoState noEgo()
{
  return EgoState{};
}
}  // namespace

TEST(ModeManagerCore, StartsInStandbyAndSafeStopsWithoutEgo)
{
  ModeManagerCore core{ModeManagerParams{}};
  const Decision d = core.update(0.0, noEgo(), absent(), absent());
  EXPECT_EQ(d.mode, Mode::STANDBY);
  EXPECT_EQ(d.route, Route::SAFE_STOP);
  EXPECT_FALSE(d.nav2_should_be_active);
}

TEST(ModeManagerCore, PromotesToAwPlanningWhenEgoAndOnroadAvailable)
{
  ModeManagerCore core{ModeManagerParams{}};
  const Decision d = core.update(0.0, egoAt(), freshValid(), absent());
  EXPECT_EQ(d.mode, Mode::AW_PLANNING);
  EXPECT_EQ(d.route, Route::ONROAD);
  EXPECT_EQ(d.transition, Transition::NONE);
  EXPECT_FALSE(d.nav2_should_be_active);
}

TEST(ModeManagerCore, StaysStandbyWhenOnroadNotYetAvailable)
{
  ModeManagerCore core{ModeManagerParams{}};
  const Decision d = core.update(0.0, egoAt(), staleSource(), absent());
  EXPECT_EQ(d.mode, Mode::STANDBY);
  EXPECT_EQ(d.route, Route::SAFE_STOP);
}

TEST(ModeManagerCore, RequestNav2EntersTransitionStillRoutingOnroad)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());  // settle in AW

  std::string msg;
  EXPECT_TRUE(core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg));

  const Decision d = core.update(0.1, egoAt(), freshValid(), absent());  // offroad not ready
  EXPECT_EQ(d.mode, Mode::AW_PLANNING);
  EXPECT_EQ(d.transition, Transition::TO_NAV2);
  EXPECT_EQ(d.route, Route::ONROAD);
  EXPECT_TRUE(d.nav2_should_be_active);  // bring nav2 up during the transition
}

TEST(ModeManagerCore, CommitsToNav2WhenOffroadValidAndContinuous)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());
  std::string msg;
  core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg);

  const Decision d = core.update(0.1, egoAt(), freshValid(), freshValid());  // offroad now ready & at ego
  EXPECT_EQ(d.mode, Mode::NAV2_OFFROAD);
  EXPECT_EQ(d.transition, Transition::NONE);
  EXPECT_EQ(d.route, Route::OFFROAD);
  EXPECT_TRUE(d.nav2_should_be_active);
}

TEST(ModeManagerCore, Nav2CommitBlockedByDiscontinuousOffroad)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());
  std::string msg;
  core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg);

  // offroad usable but its first point is 10 m away from ego -> discontinuous
  const Decision d = core.update(0.1, egoAt(0.0, 0.0), freshValid(), freshValid(10.0, 0.0));
  EXPECT_EQ(d.transition, Transition::TO_NAV2);  // not committed
  EXPECT_EQ(d.route, Route::ONROAD);
}

TEST(ModeManagerCore, ForceBypassesContinuityGuard)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());
  std::string msg;
  core.requestMode(Mode::NAV2_OFFROAD, true, 0.0, msg);  // force

  const Decision d = core.update(0.1, egoAt(0.0, 0.0), freshValid(), freshValid(10.0, 0.0));
  EXPECT_EQ(d.mode, Mode::NAV2_OFFROAD);  // committed despite the gap
  EXPECT_EQ(d.route, Route::OFFROAD);
}

TEST(ModeManagerCore, Nav2TransitionTimesOutBackToAw)
{
  ModeManagerParams p;
  p.transition_timeout_s = 2.0;
  ModeManagerCore core{p};
  core.update(0.0, egoAt(), freshValid(), absent());
  std::string msg;
  core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg);

  const Decision d = core.update(5.0, egoAt(), freshValid(), absent());  // past timeout, no offroad
  EXPECT_EQ(d.mode, Mode::AW_PLANNING);
  EXPECT_EQ(d.transition, Transition::NONE);
  EXPECT_EQ(d.route, Route::ONROAD);
  EXPECT_FALSE(d.nav2_should_be_active);
  EXPECT_FALSE(d.fault_reason.empty());
}

TEST(ModeManagerCore, SteadyAwLosingOnroadGoesSafeStop)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());  // AW
  const Decision d = core.update(0.1, egoAt(), staleSource(), absent());  // onroad lost
  EXPECT_EQ(d.mode, Mode::SAFE_STOP);
  EXPECT_EQ(d.route, Route::SAFE_STOP);
}

TEST(ModeManagerCore, RequestAwFromNav2CommitsAndDeactivatesNav2)
{
  ModeManagerCore core{ModeManagerParams{}};
  core.update(0.0, egoAt(), freshValid(), absent());
  std::string msg;
  core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg);
  core.update(0.1, egoAt(), freshValid(), freshValid());  // commit NAV2
  ASSERT_EQ(core.mode(), Mode::NAV2_OFFROAD);

  core.requestMode(Mode::AW_PLANNING, false, 1.0, msg);
  const Decision mid = core.update(1.1, egoAt(), freshValid(), freshValid());  // onroad continuous -> commit
  EXPECT_EQ(mid.mode, Mode::AW_PLANNING);
  EXPECT_EQ(mid.route, Route::ONROAD);
  EXPECT_FALSE(mid.nav2_should_be_active);
}

TEST(ModeManagerCore, RequestModeRejectsUnsupportedTargets)
{
  ModeManagerCore core{ModeManagerParams{}};
  std::string msg;
  EXPECT_FALSE(core.requestMode(Mode::STANDBY, false, 0.0, msg));
  EXPECT_FALSE(core.requestMode(Mode::SAFE_STOP, false, 0.0, msg));
  EXPECT_TRUE(core.requestMode(Mode::NAV2_OFFROAD, false, 0.0, msg));
}

TEST(ModeManagerCore, EvaluateGuardsReportsGapsAgesAndUsability)
{
  ModeManagerCore core{ModeManagerParams{}};
  const auto guard =
    core.evaluateGuards(egoAt(0.0, 0.0, 0.0), freshValid(), freshValid(10.0, 0.0, 0.0), true);
  EXPECT_TRUE(guard.target_is_offroad);
  EXPECT_TRUE(guard.onroad_usable);
  EXPECT_TRUE(guard.offroad_usable);
  EXPECT_NEAR(guard.offroad_age_s, 0.0, 1e-6);
  EXPECT_NEAR(guard.position_gap_m, 10.0, 1e-6);  // offroad first point is 10 m away
  EXPECT_FALSE(guard.continuity_ok);              // 10 m > default 2 m gap
}

TEST(ModeManagerCore, EvaluateGuardsContinuousWhenTargetNearEgo)
{
  ModeManagerCore core{ModeManagerParams{}};
  const auto guard =
    core.evaluateGuards(egoAt(0.0, 0.0, 0.0), freshValid(1.0, 0.0, 0.0), absent(), false);
  EXPECT_FALSE(guard.target_is_offroad);
  EXPECT_NEAR(guard.position_gap_m, 1.0, 1e-6);
  EXPECT_TRUE(guard.continuity_ok);  // 1 m <= 2 m
}
