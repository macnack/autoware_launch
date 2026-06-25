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

#include "autoware_nav2_offroad/occupancy_grid_relay.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

using autoware::nav2_offroad::remapOccupancy;

TEST(OccupancyGridRelay, PassesValuesThroughWhenUnknownKept)
{
  const std::vector<int8_t> in{-1, 0, 50, 100};
  EXPECT_EQ(remapOccupancy(in, false), in);
}

TEST(OccupancyGridRelay, MapsUnknownToFreeWhenEnabled)
{
  const std::vector<int8_t> in{-1, 0, 50, 100, -1};
  const std::vector<int8_t> expected{0, 0, 50, 100, 0};
  EXPECT_EQ(remapOccupancy(in, true), expected);
}
