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

#ifndef AUTOWARE_NAV2_OFFROAD__MODE_MANAGER_CORE_HPP_
#define AUTOWARE_NAV2_OFFROAD__MODE_MANAGER_CORE_HPP_

#include <string>

namespace autoware::nav2_offroad
{

// Keep STANDBY at 0 so a value-initialized Decision starts in STANDBY.
enum class Mode { STANDBY = 0, AW_PLANNING, NAV2_OFFROAD, SAFE_STOP };
enum class Transition { NONE = 0, TO_AW, TO_NAV2 };
// Which source the node should publish to /planning/trajectory.
enum class Route { NONE = 0, ONROAD, OFFROAD, SAFE_STOP };

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Pre-digested view of one trajectory input (the node fills this from ROS msgs).
struct SourceState
{
  bool present{false};            // a message has ever been received
  double age_s{1e9};              // seconds since the last message
  bool valid{false};             // passed validity checks (>= min points, finite, ...)
  Pose2d first_pose{};            // pose of the first trajectory point
  double first_velocity_mps{0.0};
};

struct EgoState
{
  bool valid{false};
  Pose2d pose{};
  double velocity_mps{0.0};
};

struct ModeManagerParams
{
  double target_trajectory_timeout_s{1.0};
  double max_position_gap_m{2.0};
  double max_yaw_gap_rad{0.5};
  double max_velocity_step_mps{1.0};
  double transition_timeout_s{5.0};
  Mode mode_on_startup{Mode::AW_PLANNING};
};

// Snapshot of the guard inputs for one evaluation, for debugging why a switch
// was (or was not) allowed. Not part of the control decision.
struct GuardDebug
{
  bool target_is_offroad{false};
  bool onroad_usable{false};
  bool offroad_usable{false};
  double onroad_age_s{0.0};
  double offroad_age_s{0.0};
  double position_gap_m{0.0};
  double yaw_gap_rad{0.0};
  double velocity_gap_mps{0.0};
  bool continuity_ok{false};
};

struct Decision
{
  Mode mode{Mode::STANDBY};
  Transition transition{Transition::NONE};
  Mode requested_mode{Mode::STANDBY};
  Route route{Route::NONE};
  bool nav2_should_be_active{false};
  std::string fault_reason{};
};

/// Pure (ROS-free) mode state machine + transition guards for
/// trajectory_mode_manager. All time is passed in as monotonic seconds so the
/// logic is fully deterministic and unit-testable.
class ModeManagerCore
{
public:
  explicit ModeManagerCore(ModeManagerParams params);

  /// Request a target mode. Returns true if accepted; `message` carries detail.
  bool requestMode(Mode target, bool force, double now_s, std::string & message);

  /// Advance the state machine with the latest world state; returns the decision.
  Decision update(
    double now_s, const EgoState & ego, const SourceState & onroad,
    const SourceState & offroad);

  /// Evaluate the guard inputs for debugging/visualization (does not mutate state).
  GuardDebug evaluateGuards(
    const EgoState & ego, const SourceState & onroad, const SourceState & offroad,
    bool target_is_offroad) const;

  Mode mode() const { return mode_; }
  Transition transition() const { return transition_; }

private:
  ModeManagerParams params_;
  Mode mode_{Mode::STANDBY};
  Transition transition_{Transition::NONE};
  Mode requested_{Mode::STANDBY};
  bool force_request_{false};
  double transition_start_s_{0.0};
};

}  // namespace autoware::nav2_offroad

#endif  // AUTOWARE_NAV2_OFFROAD__MODE_MANAGER_CORE_HPP_
