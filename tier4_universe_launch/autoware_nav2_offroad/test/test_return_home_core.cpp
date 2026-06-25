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

#include "autoware_nav2_offroad/return_home_core.hpp"

#include <gtest/gtest.h>

namespace
{
using autoware::nav2_offroad::Pose2d;
using autoware::nav2_offroad::ReturnHomeCore;
using autoware::nav2_offroad::ReturnHomeParams;

ReturnHomeCore makeCore()
{
  ReturnHomeParams p;
  p.reached_distance_m = 1.0;
  p.stopped_velocity_mps = 0.1;
  return ReturnHomeCore(p);
}

TEST(ReturnHomeCore, NoHomeByDefault)
{
  auto c = makeCore();
  EXPECT_FALSE(c.hasHome());
  EXPECT_DOUBLE_EQ(c.distanceToHome({5.0, 5.0, 0.0}), 0.0);
  EXPECT_FALSE(c.isHomeReached({0.0, 0.0, 0.0}, 0.0));
}

TEST(ReturnHomeCore, SetHomeStoresPose)
{
  auto c = makeCore();
  c.setHome({1.0, 2.0, 0.5});
  EXPECT_TRUE(c.hasHome());
  EXPECT_DOUBLE_EQ(c.home().x, 1.0);
  EXPECT_DOUBLE_EQ(c.home().y, 2.0);
}

TEST(ReturnHomeCore, ClearHomeResets)
{
  auto c = makeCore();
  c.setHome({1.0, 2.0, 0.0});
  c.clearHome();
  EXPECT_FALSE(c.hasHome());
}

TEST(ReturnHomeCore, DistanceToHomeIsEuclidean)
{
  auto c = makeCore();
  c.setHome({0.0, 0.0, 0.0});
  EXPECT_NEAR(c.distanceToHome({3.0, 4.0, 0.0}), 5.0, 1e-9);
}

TEST(ReturnHomeCore, IsHomeReachedNeedsCloseAndStopped)
{
  auto c = makeCore();
  c.setHome({0.0, 0.0, 0.0});
  EXPECT_TRUE(c.isHomeReached({0.5, 0.0, 0.0}, 0.05));   // close + stopped
  EXPECT_FALSE(c.isHomeReached({5.0, 0.0, 0.0}, 0.05));  // far
  EXPECT_FALSE(c.isHomeReached({0.5, 0.0, 0.0}, 1.0));   // moving
}
}  // namespace
