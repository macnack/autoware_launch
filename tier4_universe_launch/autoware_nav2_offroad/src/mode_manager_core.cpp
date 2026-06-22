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

#include "autoware_nav2_offroad/mode_manager_core.hpp"

#include <cmath>
#include <string>

namespace autoware::nav2_offroad
{
namespace
{
bool usable(const SourceState & s, const ModeManagerParams & p)
{
  return s.present && s.valid && s.age_s <= p.target_trajectory_timeout_s;
}

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

bool continuous(const EgoState & ego, const SourceState & s, const ModeManagerParams & p)
{
  const double position_gap =
    std::hypot(ego.pose.x - s.first_pose.x, ego.pose.y - s.first_pose.y);
  const double yaw_gap = std::fabs(normalizeAngle(ego.pose.yaw - s.first_pose.yaw));
  const double velocity_gap = std::fabs(ego.velocity_mps - s.first_velocity_mps);
  return position_gap <= p.max_position_gap_m && yaw_gap <= p.max_yaw_gap_rad &&
         velocity_gap <= p.max_velocity_step_mps;
}

// Route to publish while a transition is in flight: keep the currently-committed
// source until the target is proven, falling back to a safe stop if it is gone.
Route holdRoute(
  Mode mode, const SourceState & onroad, const SourceState & offroad,
  const ModeManagerParams & p)
{
  if (mode == Mode::AW_PLANNING) {
    return usable(onroad, p) ? Route::ONROAD : Route::SAFE_STOP;
  }
  if (mode == Mode::NAV2_OFFROAD) {
    return usable(offroad, p) ? Route::OFFROAD : Route::SAFE_STOP;
  }
  return Route::SAFE_STOP;
}
}  // namespace

ModeManagerCore::ModeManagerCore(ModeManagerParams params) : params_(params) {}

bool ModeManagerCore::requestMode(Mode target, bool force, double now_s, std::string & message)
{
  if (target != Mode::AW_PLANNING && target != Mode::NAV2_OFFROAD) {
    message = "unsupported target mode";
    return false;
  }
  requested_ = target;
  if (target == mode_ && transition_ == Transition::NONE) {
    message = "already in requested mode";
    return true;
  }
  transition_ = (target == Mode::NAV2_OFFROAD) ? Transition::TO_NAV2 : Transition::TO_AW;
  transition_start_s_ = now_s;
  force_request_ = force;
  message = "transition started";
  return true;
}

Decision ModeManagerCore::update(
  double now_s, const EgoState & ego, const SourceState & onroad, const SourceState & offroad)
{
  Decision d;

  // Without localization nothing can be done safely.
  if (!ego.valid) {
    mode_ = Mode::STANDBY;
    transition_ = Transition::NONE;
    d.mode = Mode::STANDBY;
    d.requested_mode = requested_;
    d.route = Route::SAFE_STOP;
    d.fault_reason = "localization unavailable";
    return d;
  }

  // Leave STANDBY into the startup mode once its source is usable, unless a
  // transition has already been requested.
  if (mode_ == Mode::STANDBY && transition_ == Transition::NONE) {
    if (params_.mode_on_startup == Mode::NAV2_OFFROAD && usable(offroad, params_)) {
      mode_ = Mode::NAV2_OFFROAD;
      requested_ = Mode::NAV2_OFFROAD;
    } else if (usable(onroad, params_)) {
      mode_ = Mode::AW_PLANNING;
      requested_ = Mode::AW_PLANNING;
    } else {
      d.mode = Mode::STANDBY;
      d.requested_mode = requested_;
      d.route = Route::SAFE_STOP;
      d.fault_reason = "awaiting planning source";
      return d;
    }
  }

  // Active transition handling: commit when the target is proven, abort on timeout.
  if (transition_ == Transition::TO_NAV2) {
    if (usable(offroad, params_) && (force_request_ || continuous(ego, offroad, params_))) {
      mode_ = Mode::NAV2_OFFROAD;
      transition_ = Transition::NONE;
      force_request_ = false;
    } else if (now_s - transition_start_s_ > params_.transition_timeout_s) {
      transition_ = Transition::NONE;
      force_request_ = false;
      if (usable(onroad, params_)) {
        mode_ = Mode::AW_PLANNING;
        requested_ = Mode::AW_PLANNING;
        d.fault_reason = "nav2 transition timed out, reverted to on-road";
      } else {
        mode_ = Mode::SAFE_STOP;
        d.fault_reason = "nav2 transition timed out, no on-road fallback";
      }
    } else {
      d.mode = mode_;
      d.transition = Transition::TO_NAV2;
      d.requested_mode = requested_;
      d.route = holdRoute(mode_, onroad, offroad, params_);
      d.nav2_should_be_active = true;
      return d;
    }
  } else if (transition_ == Transition::TO_AW) {
    if (usable(onroad, params_) && (force_request_ || continuous(ego, onroad, params_))) {
      mode_ = Mode::AW_PLANNING;
      transition_ = Transition::NONE;
      force_request_ = false;
    } else if (now_s - transition_start_s_ > params_.transition_timeout_s) {
      transition_ = Transition::NONE;
      force_request_ = false;
      if (usable(offroad, params_)) {
        mode_ = Mode::NAV2_OFFROAD;
        requested_ = Mode::NAV2_OFFROAD;
        d.fault_reason = "on-road transition timed out, stayed off-road";
      } else {
        mode_ = Mode::SAFE_STOP;
        d.fault_reason = "on-road transition timed out, no off-road fallback";
      }
    } else {
      d.mode = mode_;
      d.transition = Transition::TO_AW;
      d.requested_mode = requested_;
      d.route = holdRoute(mode_, onroad, offroad, params_);
      d.nav2_should_be_active = true;  // nav2 still needed until commit
      return d;
    }
  }

  // Steady-state evaluation.
  switch (mode_) {
    case Mode::AW_PLANNING:
      if (usable(onroad, params_)) {
        d.route = Route::ONROAD;
      } else {
        mode_ = Mode::SAFE_STOP;
        d.fault_reason = "on-road trajectory lost";
      }
      break;
    case Mode::NAV2_OFFROAD:
      if (usable(offroad, params_)) {
        d.route = Route::OFFROAD;
        d.nav2_should_be_active = true;
      } else {
        mode_ = Mode::SAFE_STOP;
        d.fault_reason = "off-road trajectory lost";
      }
      break;
    case Mode::SAFE_STOP:
    case Mode::STANDBY:
    default:
      break;
  }

  if (mode_ == Mode::SAFE_STOP || mode_ == Mode::STANDBY) {
    d.route = Route::SAFE_STOP;
  }

  d.mode = mode_;
  d.transition = Transition::NONE;
  d.requested_mode = requested_;
  return d;
}

}  // namespace autoware::nav2_offroad
