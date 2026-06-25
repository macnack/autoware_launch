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

#ifndef AUTOWARE_NAV2_OFFROAD__RETURN_HOME_CORE_HPP_
#define AUTOWARE_NAV2_OFFROAD__RETURN_HOME_CORE_HPP_

#include "autoware_nav2_offroad/mode_manager_core.hpp"  // Pose2d

namespace autoware::nav2_offroad
{
struct ReturnHomeParams
{
  double reached_distance_m{1.0};
  double stopped_velocity_mps{0.1};
};

class ReturnHomeCore
{
public:
  explicit ReturnHomeCore(ReturnHomeParams params) : params_(params) {}

  void setHome(const Pose2d & home)
  {
    home_ = home;
    has_home_ = true;
  }
  void clearHome()
  {
    has_home_ = false;
    home_ = Pose2d{};
  }
  bool hasHome() const { return has_home_; }
  Pose2d home() const { return home_; }
  double distanceToHome(const Pose2d & ego) const;
  bool isHomeReached(const Pose2d & ego, double velocity_mps) const;

private:
  ReturnHomeParams params_;
  bool has_home_{false};
  Pose2d home_{};
};
}  // namespace autoware::nav2_offroad
#endif  // AUTOWARE_NAV2_OFFROAD__RETURN_HOME_CORE_HPP_
