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

#include "autoware_nav2_offroad/gear_arbiter.hpp"

#include <gtest/gtest.h>

namespace an = autoware::nav2_offroad;

namespace
{
an::GearArbiter makeArbiter() { return an::GearArbiter(0.1, 0.05); }
}  // namespace

TEST(GearArbiter, forward_cmd_in_drive_passes_through)
{
  auto a = makeArbiter();
  const auto out = a.update(1.2, 1.0);
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 1.2);
}

TEST(GearArbiter, deadband_holds_gear_and_outputs_zero)
{
  auto a = makeArbiter();
  const auto out = a.update(0.02, 0.0);  // |cmd| < 0.05 deadband
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
}

TEST(GearArbiter, flip_while_rolling_holds_gear_and_commands_zero)
{
  auto a = makeArbiter();
  a.update(1.0, 1.0);                     // established DRIVE, rolling
  const auto out = a.update(-1.0, 0.8);   // reverse requested, still rolling
  EXPECT_EQ(out.gear, an::Gear::DRIVE);   // NO gear change while |speed| >= 0.1
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
}

TEST(GearArbiter, flip_shifts_only_after_stop_then_passes_reverse)
{
  auto a = makeArbiter();
  a.update(1.0, 1.0);
  a.update(-1.0, 0.5);                    // hold: still rolling
  a.update(-1.0, 0.2);                    // hold: still rolling
  const auto out = a.update(-1.0, 0.05);  // |speed| < 0.1 -> shift + pass
  EXPECT_EQ(out.gear, an::Gear::REVERSE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, -1.0);
}

TEST(GearArbiter, reverse_to_drive_is_symmetric)
{
  auto a = makeArbiter();
  a.update(-1.0, 0.0);                    // shift to REVERSE at standstill
  a.update(-1.0, -0.8);                   // rolling backwards
  auto out = a.update(0.8, -0.5);         // forward requested while rolling back
  EXPECT_EQ(out.gear, an::Gear::REVERSE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
  out = a.update(0.8, -0.04);             // nearly stopped -> shift + pass
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.8);
}

TEST(GearArbiter, no_gear_change_is_ever_emitted_while_rolling)
{
  auto a = makeArbiter();
  a.update(1.0, 1.5);
  for (double sp = 1.5; sp >= 0.1; sp -= 0.1) {
    const auto out = a.update(-1.0, sp);
    EXPECT_EQ(out.gear, an::Gear::DRIVE) << "shifted at speed " << sp;
  }
}

TEST(GearArbiter, flip_debounce_ignores_transient_direction_dither)
{
  // persistence = 4: a flip request must persist 4 consecutive updates before
  // the shift is honored, even at standstill (MPPI dither must not thrash gears).
  an::GearArbiter a(0.1, 0.05, 4);
  a.update(1.0, 0.0);                     // DRIVE established
  // 3 reverse requests, then forward again: NO shift.
  for (int i = 0; i < 3; ++i) {
    const auto out = a.update(-0.5, 0.0);
    EXPECT_EQ(out.gear, an::Gear::DRIVE);
    EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
  }
  auto out = a.update(0.8, 0.0);
  EXPECT_EQ(out.gear, an::Gear::DRIVE);   // dither absorbed, still DRIVE
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.8);
  // A PERSISTENT reverse request (4 consecutive) shifts on the 4th.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(a.update(-0.5, 0.0).gear, an::Gear::DRIVE);
  }
  out = a.update(-0.5, 0.0);
  EXPECT_EQ(out.gear, an::Gear::REVERSE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, -0.5);
}

TEST(GearArbiter, flip_debounce_counter_resets_on_same_direction)
{
  an::GearArbiter a(0.1, 0.05, 3);
  a.update(1.0, 0.0);
  a.update(-0.5, 0.0);
  a.update(-0.5, 0.0);
  a.update(1.0, 0.0);                     // back to forward -> counter resets
  a.update(-0.5, 0.0);
  const auto out = a.update(-0.5, 0.0);   // only 2 consecutive -> still DRIVE
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
}
