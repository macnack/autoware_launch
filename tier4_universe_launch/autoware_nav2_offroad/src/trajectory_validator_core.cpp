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

#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace autoware::nav2_offroad
{
namespace
{
ValidationResult fail(
  FailedCheck check, std::size_t index, double value, double limit, std::string reason)
{
  ValidationResult r;
  r.feasible = false;
  r.check = check;
  r.point_index = index;
  r.worst_value = value;
  r.limit = limit;
  r.reason = std::move(reason);
  return r;
}

bool allFinite(const TrajPoint & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.yaw) &&
         std::isfinite(q.velocity_mps) && std::isfinite(q.acceleration_mps2);
}
}  // namespace

std::string toString(FailedCheck check)
{
  switch (check) {
    case FailedCheck::NONE:
      return "none";
    case FailedCheck::TOO_FEW_POINTS:
      return "too_few_points";
    case FailedCheck::NON_FINITE:
      return "non_finite";
    case FailedCheck::VELOCITY:
      return "velocity";
    case FailedCheck::LONGITUDINAL_ACCEL:
      return "longitudinal_accel";
    case FailedCheck::CURVATURE:
      return "curvature";
    case FailedCheck::LATERAL_ACCEL:
      return "lateral_accel";
    case FailedCheck::CONTINUITY:
      return "continuity";
  }
  return "unknown";
}

TrajectoryValidatorCore::TrajectoryValidatorCore(ValidatorParams params) : params_(params) {}

ValidationResult TrajectoryValidatorCore::validate(
  const std::vector<TrajPoint> & traj, const EgoState & ego) const
{
  (void)ego;
  const ValidatorParams & p = params_;

  // 1. Structural.
  if (traj.size() < p.min_points) {
    return fail(
      FailedCheck::TOO_FEW_POINTS, 0, static_cast<double>(traj.size()),
      static_cast<double>(p.min_points),
      "trajectory has " + std::to_string(traj.size()) + " points, need at least " +
        std::to_string(p.min_points));
  }

  // 2. Finite.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    if (!allFinite(traj[i])) {
      return fail(
        FailedCheck::NON_FINITE, i, 0.0, 0.0, "non-finite value at point " + std::to_string(i));
    }
  }

  // 3. Velocity.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double v = std::fabs(traj[i].velocity_mps);
    if (v > p.max_velocity_mps) {
      return fail(
        FailedCheck::VELOCITY, i, v, p.max_velocity_mps,
        "speed " + std::to_string(v) + " m/s exceeds max " + std::to_string(p.max_velocity_mps) +
          " at point " + std::to_string(i));
    }
  }

  // 4. Longitudinal acceleration.
  for (std::size_t i = 0; i < traj.size(); ++i) {
    const double a = std::fabs(traj[i].acceleration_mps2);
    if (a > p.max_longitudinal_accel_mps2) {
      return fail(
        FailedCheck::LONGITUDINAL_ACCEL, i, a, p.max_longitudinal_accel_mps2,
        "|accel| " + std::to_string(a) + " m/s^2 exceeds max " +
          std::to_string(p.max_longitudinal_accel_mps2) + " at point " + std::to_string(i));
    }
  }

  return ValidationResult{};  // feasible
}

}  // namespace autoware::nav2_offroad
