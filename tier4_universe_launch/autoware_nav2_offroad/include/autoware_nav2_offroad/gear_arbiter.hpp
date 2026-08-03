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

#ifndef AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_
#define AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_

namespace autoware::nav2_offroad
{
enum class Gear { DRIVE, REVERSE };

struct GearArbiterOut
{
  Gear gear{Gear::DRIVE};
  double velocity_mps{0.0};
};

/// Stop-and-shift gear arbitration for a reverse-capable cmd_vel bridge.
/// On a commanded direction flip, outputs zero velocity and holds the current
/// gear until the vehicle is nearly stopped (|speed| < stop_threshold), then
/// shifts and passes the new-direction command through. Pure unit: no ROS.
class GearArbiter
{
public:
  /// flip_persistence_updates: a direction-flip request must persist this many
  /// consecutive update() calls before the shift is honored ("negative reward"
  /// for gear changes at the execution level — MPPI dither must not thrash
  /// gears; each real shift already costs a full stop). 1 = no debounce.
  GearArbiter(double stop_threshold_mps, double deadband_mps, int flip_persistence_updates = 1)
  : stop_threshold_mps_(stop_threshold_mps),
    deadband_mps_(deadband_mps),
    flip_persistence_updates_(flip_persistence_updates)
  {
  }

  GearArbiterOut update(double cmd_v_mps, double vehicle_speed_mps);

private:
  Gear gear_{Gear::DRIVE};
  double stop_threshold_mps_;
  double deadband_mps_;
  int flip_persistence_updates_;
  int flip_request_count_{0};
};
}  // namespace autoware::nav2_offroad
#endif  // AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_
