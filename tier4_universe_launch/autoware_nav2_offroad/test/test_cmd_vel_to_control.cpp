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

#include "autoware_nav2_offroad/cmd_vel_to_control.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::nav2_offroad::BicycleParams;
using autoware::nav2_offroad::twistToControl;

TEST(TwistToControl, StraightLineZeroSteer)
{
  BicycleParams p;
  const auto c = twistToControl(2.0, 0.0, p, 0.0);
  EXPECT_DOUBLE_EQ(c.velocity_mps, 2.0);
  EXPECT_NEAR(c.steering_tire_angle_rad, 0.0, 1e-9);
}

TEST(TwistToControl, BicycleModelSteer)
{
  BicycleParams p; p.wheelbase_m = 2.0;
  // delta = atan(L * omega / v) = atan(2 * 0.5 / 2.0) = atan(0.5)
  const auto c = twistToControl(2.0, 0.5, p, 0.0);
  EXPECT_NEAR(c.steering_tire_angle_rad, std::atan(0.5), 1e-9);
}

TEST(TwistToControl, ClampsToMaxSteer)
{
  BicycleParams p; p.wheelbase_m = 2.0; p.max_steer_rad = 0.3;
  const auto c = twistToControl(0.5, 5.0, p, 0.0);
  EXPECT_NEAR(c.steering_tire_angle_rad, 0.3, 1e-9);
}

TEST(TwistToControl, ZeroSpeedHoldsLastSteer)
{
  BicycleParams p; p.min_speed_for_steer_mps = 0.1;
  const auto c = twistToControl(0.0, 1.0, p, 0.25);
  EXPECT_DOUBLE_EQ(c.steering_tire_angle_rad, 0.25);  // holds last
  EXPECT_DOUBLE_EQ(c.velocity_mps, 0.0);
}
TEST(TwistToControl, ReverseVelocityProducesSignConsistentSteer)
{
  BicycleParams p; p.wheelbase_m = 2.0; p.max_steer_rad = 1.0;
  // Reverse motion: atan(2.0 * 0.5 / -2.0) = atan(-0.5), steer must be negative.
  const auto c = twistToControl(-2.0, 0.5, p, 0.0);
  EXPECT_DOUBLE_EQ(c.velocity_mps, -2.0);
  EXPECT_NEAR(c.steering_tire_angle_rad, std::atan(2.0 * 0.5 / -2.0), 1e-9);
  // Reverse low-speed: |v| < min_speed_for_steer_mps, hold last steer.
  const auto c2 = twistToControl(-0.05, 1.0, p, 0.25);
  EXPECT_DOUBLE_EQ(c2.steering_tire_angle_rad, 0.25);
  EXPECT_DOUBLE_EQ(c2.velocity_mps, -0.05);
}

using autoware::nav2_offroad::computeAccelCommand;

TEST(ComputeAccelCommand, ForwardTracksVelocityError)
{
  // target 2.0, measured 1.0, gain 1.5 -> +1.5 (accelerate)
  EXPECT_DOUBLE_EQ(computeAccelCommand(2.0, 1.0, false, 1.5, 3.0), 1.5);
  // target 0.0, measured 2.0 -> -3.0 (brake, clamped to the limit)
  EXPECT_DOUBLE_EQ(computeAccelCommand(0.0, 2.0, false, 1.5, 3.0), -3.0);
}

TEST(ComputeAccelCommand, ClampsToLimit)
{
  EXPECT_DOUBLE_EQ(computeAccelCommand(10.0, 0.0, false, 1.5, 3.0), 3.0);
  EXPECT_DOUBLE_EQ(computeAccelCommand(-10.0, 0.0, true, 1.5, 3.0), 3.0);
}

TEST(ComputeAccelCommand, ReverseGearUsesGearFrame)
{
  // ACC_GEARED interfaces: in REVERSE gear, positive acceleration speeds the
  // vehicle up IN THE GEAR DIRECTION (backwards). The signed-frame velocity
  // error must be flipped in reverse — a naive signed-frame tracker produced
  // a runaway (brake command accelerated the vehicle backwards).
  // Backing slower than target (-0.5 vs -1.5): speed up backwards -> +accel.
  EXPECT_DOUBLE_EQ(computeAccelCommand(-1.5, -0.5, true, 1.5, 3.0), 1.5);
  // Backing FASTER than target (-3.0 vs -1.5): brake -> negative accel.
  EXPECT_DOUBLE_EQ(computeAccelCommand(-1.5, -3.0, true, 1.5, 3.0), -2.25);
  // Commanded stop while rolling backwards: brake.
  EXPECT_LT(computeAccelCommand(0.0, -2.0, true, 1.5, 3.0), 0.0);
}

using autoware::nav2_offroad::isFullStopCommand;

TEST(IsFullStopCommand, TrueWhenBothVelocityAndOmegaAreZero)
{
  // What the controller (RPP/MPPI) publishes on goal arrival: a genuine
  // zero-Twist, not just a transiently low speed mid-maneuver.
  EXPECT_TRUE(isFullStopCommand(0.0, 0.0, 1e-3));
  EXPECT_TRUE(isFullStopCommand(0.0005, -0.0005, 1e-3));  // within epsilon
}

TEST(IsFullStopCommand, FalseWhenEitherIsNonZero)
{
  // Near-zero speed but still turning (e.g. approaching a cusp mid-maneuver):
  // NOT a full stop — the bridge must keep holding the last steer here.
  EXPECT_FALSE(isFullStopCommand(0.0, 0.5, 1e-3));
  // Commanded forward motion with straight steering: not a stop either.
  EXPECT_FALSE(isFullStopCommand(1.0, 0.0, 1e-3));
}
}  // namespace
