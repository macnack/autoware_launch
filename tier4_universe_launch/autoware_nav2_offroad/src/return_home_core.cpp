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

#include <cmath>

namespace autoware::nav2_offroad
{
double ReturnHomeCore::distanceToHome(const Pose2d & ego) const
{
  if (!has_home_) return 0.0;
  return std::hypot(ego.x - home_.x, ego.y - home_.y);
}

bool ReturnHomeCore::isHomeReached(const Pose2d & ego, double velocity_mps) const
{
  if (!has_home_) return false;
  return distanceToHome(ego) <= params_.reached_distance_m &&
         std::abs(velocity_mps) <= params_.stopped_velocity_mps;
}
}  // namespace autoware::nav2_offroad
