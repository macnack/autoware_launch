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

#ifndef AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_
#define AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_

#include "autoware_nav2_offroad/mode_manager_core.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace autoware::nav2_offroad
{

// One trajectory sample, digested from autoware_planning_msgs::TrajectoryPoint.
struct TrajPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double velocity_mps{0.0};
  double acceleration_mps2{0.0};
};

enum class FailedCheck {
  NONE = 0,
  TOO_FEW_POINTS,
  NON_FINITE,
  VELOCITY,
  LONGITUDINAL_ACCEL,
  CURVATURE,
  LATERAL_ACCEL,
  CONTINUITY
};

std::string toString(FailedCheck check);

struct ValidatorParams
{
  std::size_t min_points{2};
  double max_velocity_mps{5.0};
  double max_longitudinal_accel_mps2{2.0};
  double max_lateral_accel_mps2{2.0};
  double max_curvature_1pm{1.0};
  // Continuity thresholds (same defaults as ModeManagerParams).
  double max_position_gap_m{2.0};
  double max_yaw_gap_rad{0.5};
  double max_velocity_step_mps{1.0};
};

struct ValidationResult
{
  bool feasible{true};
  FailedCheck check{FailedCheck::NONE};
  std::size_t point_index{0};
  double worst_value{0.0};
  double limit{0.0};
  std::string reason{};
};

/// Pure (ROS-free) feasibility gate for an off-road trajectory. Deterministic
/// and unit-testable: all inputs are digested plain data.
class TrajectoryValidatorCore
{
public:
  explicit TrajectoryValidatorCore(ValidatorParams params);

  ValidationResult validate(const std::vector<TrajPoint> & traj, const EgoState & ego) const;

private:
  ValidatorParams params_;
};

}  // namespace autoware::nav2_offroad

#endif  // AUTOWARE_NAV2_OFFROAD__TRAJECTORY_VALIDATOR_CORE_HPP_
