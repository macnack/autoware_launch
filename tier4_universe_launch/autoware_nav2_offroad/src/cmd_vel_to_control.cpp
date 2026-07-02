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

#include <algorithm>
#include <cmath>

namespace autoware::nav2_offroad
{
ControlOut twistToControl(
  double v_mps, double omega_radps, const BicycleParams & p, double last_steer_rad)
{
  ControlOut out;
  out.velocity_mps = v_mps;
  if (std::abs(v_mps) < p.min_speed_for_steer_mps) {
    out.steering_tire_angle_rad = last_steer_rad;  // avoid divide-by-~0; hold
    return out;
  }
  // Negative v (reverse) naturally inverts the steer sign; Nav2 supplies a sign-consistent omega.
  double delta = std::atan(p.wheelbase_m * omega_radps / v_mps);
  delta = std::clamp(delta, -p.max_steer_rad, p.max_steer_rad);
  out.steering_tire_angle_rad = delta;
  return out;
}
}  // namespace autoware::nav2_offroad
