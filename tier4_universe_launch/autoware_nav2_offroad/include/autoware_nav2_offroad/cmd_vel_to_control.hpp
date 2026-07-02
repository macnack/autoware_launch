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

#ifndef AUTOWARE_NAV2_OFFROAD__CMD_VEL_TO_CONTROL_HPP_
#define AUTOWARE_NAV2_OFFROAD__CMD_VEL_TO_CONTROL_HPP_

namespace autoware::nav2_offroad
{
struct BicycleParams
{
  double wheelbase_m{2.7};
  double max_steer_rad{0.6};
  double min_speed_for_steer_mps{0.1};
};
struct ControlOut
{
  double velocity_mps{0.0};
  double steering_tire_angle_rad{0.0};
};
ControlOut twistToControl(
  double v_mps, double omega_radps, const BicycleParams & p, double last_steer_rad);
}  // namespace autoware::nav2_offroad
#endif  // AUTOWARE_NAV2_OFFROAD__CMD_VEL_TO_CONTROL_HPP_
