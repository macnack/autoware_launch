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

#include <cmath>

namespace autoware::nav2_offroad
{
GearArbiterOut GearArbiter::update(double cmd_v_mps, double vehicle_speed_mps)
{
  // Deadband: no meaningful command -> hold gear, command stop.
  if (std::abs(cmd_v_mps) < deadband_mps_) {
    return {gear_, 0.0};
  }
  const Gear desired = cmd_v_mps > 0.0 ? Gear::DRIVE : Gear::REVERSE;
  if (desired == gear_) {
    return {gear_, cmd_v_mps};
  }
  // Direction flip requested: hold current gear + command stop until nearly
  // stationary, then shift and pass the new-direction command through.
  if (std::abs(vehicle_speed_mps) >= stop_threshold_mps_) {
    return {gear_, 0.0};
  }
  gear_ = desired;
  return {gear_, cmd_v_mps};
}
}  // namespace autoware::nav2_offroad
