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
/// Below p.min_speed_for_steer_mps the steering angle is 0 — the geometry
/// (atan(L*omega/v)) is undefined/unstable near v=0, and the vehicle isn't
/// moving anyway, so no steering command is meaningful.
ControlOut twistToControl(double v_mps, double omega_radps, const BicycleParams & p);

/// Longitudinal acceleration command for velocity-tracking on acceleration-driven
/// vehicle interfaces (e.g. simple_planning_simulator ACC_GEARED, most real
/// interfaces): a Control message whose acceleration stays 0 never moves the
/// vehicle, whatever its velocity field says. P-law on the signed velocity error,
/// clamped to +-accel_limit. In REVERSE gear the interface interprets positive
/// acceleration as "speed up in the gear direction" (backwards), so the
/// signed-frame error is flipped into the gear frame.
double computeAccelCommand(
  double v_target_mps, double v_measured_mps, bool reverse_gear, double gain,
  double accel_limit_mps2);
}  // namespace autoware::nav2_offroad
#endif  // AUTOWARE_NAV2_OFFROAD__CMD_VEL_TO_CONTROL_HPP_
