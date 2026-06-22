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
#include "autoware_nav2_offroad/trajectory_builder.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <autoware_nav2_offroad_msgs/msg/trajectory_mode_state.hpp>
#include <autoware_nav2_offroad_msgs/srv/change_trajectory_mode.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

namespace autoware::nav2_offroad
{
namespace
{
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TrajectoryModeState = autoware_nav2_offroad_msgs::msg::TrajectoryModeState;
using ChangeTrajectoryMode = autoware_nav2_offroad_msgs::srv::ChangeTrajectoryMode;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;

std::string modeToString(const Mode mode)
{
  switch (mode) {
    case Mode::AW_PLANNING:
      return TrajectoryModeState::MODE_AW_PLANNING;
    case Mode::NAV2_OFFROAD:
      return TrajectoryModeState::MODE_NAV2_OFFROAD;
    case Mode::SAFE_STOP:
      return TrajectoryModeState::MODE_SAFE_STOP;
    case Mode::STANDBY:
    default:
      return TrajectoryModeState::MODE_STANDBY;
  }
}

std::string transitionToString(const Transition transition)
{
  switch (transition) {
    case Transition::TO_AW:
      return TrajectoryModeState::TRANSITION_TO_AW;
    case Transition::TO_NAV2:
      return TrajectoryModeState::TRANSITION_TO_NAV2;
    case Transition::NONE:
    default:
      return TrajectoryModeState::TRANSITION_NONE;
  }
}

std::optional<Mode> modeFromString(const std::string & name)
{
  if (name == TrajectoryModeState::MODE_AW_PLANNING) {
    return Mode::AW_PLANNING;
  }
  if (name == TrajectoryModeState::MODE_NAV2_OFFROAD) {
    return Mode::NAV2_OFFROAD;
  }
  return std::nullopt;
}
}  // namespace

class TrajectoryModeManagerNode : public rclcpp::Node
{
public:
  TrajectoryModeManagerNode() : Node("trajectory_mode_manager"), diagnostics_(this)
  {
    ModeManagerParams params;
    params.target_trajectory_timeout_s =
      declare_parameter<double>("target_trajectory_timeout_s", 1.0);
    params.max_position_gap_m = declare_parameter<double>("max_position_gap_m", 2.0);
    params.max_yaw_gap_rad = declare_parameter<double>("max_yaw_gap_rad", 0.5);
    params.max_velocity_step_mps = declare_parameter<double>("max_velocity_step_mps", 1.0);
    params.transition_timeout_s = declare_parameter<double>("transition_timeout_s", 5.0);
    const auto startup = declare_parameter<std::string>("mode_on_startup", "AW_PLANNING");
    params.mode_on_startup =
      (startup == "NAV2_OFFROAD") ? Mode::NAV2_OFFROAD : Mode::AW_PLANNING;

    min_valid_points_ =
      static_cast<size_t>(std::max(static_cast<int>(declare_parameter<int>("min_valid_points", 3)), 2));
    manage_nav2_ = declare_parameter<bool>("manage_nav2_lifecycle", true);
    const auto lifecycle_service =
      declare_parameter<std::string>("lifecycle_manager_service", "/lifecycle_manager_navigation/manage_nodes");
    const double publish_rate_hz = std::max(declare_parameter<double>("publish_rate_hz", 10.0), 1.0);

    core_ = std::make_unique<ModeManagerCore>(params);
    builder_ = std::make_unique<TrajectoryBuilder>(TrajectoryBuilderParams{});

    output_publisher_ = create_publisher<Trajectory>("output/trajectory", rclcpp::QoS{1});
    status_publisher_ =
      create_publisher<TrajectoryModeState>("~/status", rclcpp::QoS{1}.transient_local());

    onroad_subscription_ = create_subscription<Trajectory>(
      "input/onroad/trajectory", rclcpp::QoS{1},
      [this](Trajectory::ConstSharedPtr msg) { latest_onroad_ = *msg; });
    offroad_subscription_ = create_subscription<Trajectory>(
      "input/offroad/trajectory", rclcpp::QoS{1},
      [this](Trajectory::ConstSharedPtr msg) { latest_offroad_ = *msg; });
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { latest_odometry_ = *msg; });

    change_mode_service_ = create_service<ChangeTrajectoryMode>(
      "~/change_mode",
      std::bind(
        &TrajectoryModeManagerNode::onChangeMode, this, std::placeholders::_1,
        std::placeholders::_2));

    if (manage_nav2_) {
      nav2_lifecycle_client_ = create_client<ManageLifecycleNodes>(lifecycle_service);
    }

    diagnostics_.setHardwareID("trajectory_mode_manager");
    diagnostics_.add("mode", this, &TrajectoryModeManagerNode::produceDiagnostics);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(period, std::bind(&TrajectoryModeManagerNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "trajectory_mode_manager started (startup mode: %s)", startup.c_str());
  }

private:
  SourceState toSourceState(const std::optional<Trajectory> & trajectory) const
  {
    SourceState state;
    if (!trajectory) {
      return state;
    }
    state.present = true;
    state.age_s = (now() - rclcpp::Time(trajectory->header.stamp)).seconds();
    state.valid = trajectory->points.size() >= min_valid_points_;
    if (!trajectory->points.empty()) {
      const auto & p = trajectory->points.front();
      state.first_pose.x = p.pose.position.x;
      state.first_pose.y = p.pose.position.y;
      state.first_pose.yaw = tf2::getYaw(p.pose.orientation);
      state.first_velocity_mps = p.longitudinal_velocity_mps;
    }
    return state;
  }

  EgoState toEgoState() const
  {
    EgoState ego;
    if (!latest_odometry_) {
      return ego;
    }
    ego.valid = true;
    ego.pose.x = latest_odometry_->pose.pose.position.x;
    ego.pose.y = latest_odometry_->pose.pose.position.y;
    ego.pose.yaw = tf2::getYaw(latest_odometry_->pose.pose.orientation);
    ego.velocity_mps = latest_odometry_->twist.twist.linear.x;
    return ego;
  }

  void onTimer()
  {
    const Decision decision = core_->update(
      now().seconds(), toEgoState(), toSourceState(latest_onroad_),
      toSourceState(latest_offroad_));

    publishTrajectory(decision.route);
    syncNav2Lifecycle(decision.nav2_should_be_active);
    publishStatus(decision);
    last_decision_ = decision;
    diagnostics_.force_update();
  }

  void publishTrajectory(const Route route)
  {
    if (route == Route::ONROAD && latest_onroad_) {
      output_publisher_->publish(*latest_onroad_);
      return;
    }
    if (route == Route::OFFROAD && latest_offroad_) {
      output_publisher_->publish(*latest_offroad_);
      return;
    }
    output_publisher_->publish(builder_->createStopTrajectory(now(), latest_odometry_, std::nullopt));
  }

  void syncNav2Lifecycle(const bool should_be_active)
  {
    if (!manage_nav2_ || !nav2_lifecycle_client_) {
      return;
    }
    if (nav2_active_commanded_ && *nav2_active_commanded_ == should_be_active) {
      return;
    }
    if (!nav2_lifecycle_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = should_be_active ? ManageLifecycleNodes::Request::RESUME
                                        : ManageLifecycleNodes::Request::PAUSE;
    nav2_lifecycle_client_->async_send_request(request);
    nav2_active_commanded_ = should_be_active;
  }

  void publishStatus(const Decision & decision)
  {
    TrajectoryModeState status;
    status.stamp = now();
    status.current_mode = modeToString(decision.mode);
    status.requested_mode = modeToString(decision.requested_mode);
    status.transition = transitionToString(decision.transition);
    status.active_planner = routeToString(decision.route);
    status.fault_reason = decision.fault_reason;
    status_publisher_->publish(status);
  }

  static std::string routeToString(const Route route)
  {
    switch (route) {
      case Route::ONROAD:
        return "onroad";
      case Route::OFFROAD:
        return "offroad";
      case Route::SAFE_STOP:
        return "safe_stop";
      case Route::NONE:
      default:
        return "none";
    }
  }

  void onChangeMode(
    const ChangeTrajectoryMode::Request::SharedPtr request,
    const ChangeTrajectoryMode::Response::SharedPtr response)
  {
    const auto target = modeFromString(request->target_mode);
    if (!target) {
      response->accepted = false;
      response->current_mode = modeToString(core_->mode());
      response->message = "unsupported target mode: " + request->target_mode;
      return;
    }
    std::string message;
    response->accepted = core_->requestMode(*target, request->force, now().seconds(), message);
    response->current_mode = modeToString(core_->mode());
    response->message = message;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto & d = last_decision_;
    if (d.mode == Mode::SAFE_STOP) {
      stat.summary(DiagnosticStatus::ERROR, "safe stop: " + d.fault_reason);
    } else if (d.transition != Transition::NONE || d.mode == Mode::STANDBY) {
      stat.summary(DiagnosticStatus::WARN, transitionToString(d.transition));
    } else {
      stat.summary(DiagnosticStatus::OK, modeToString(d.mode));
    }
    stat.add("current_mode", modeToString(d.mode));
    stat.add("requested_mode", modeToString(d.requested_mode));
    stat.add("transition", transitionToString(d.transition));
    stat.add("route", routeToString(d.route));
    stat.add("nav2_should_be_active", d.nav2_should_be_active);
  }

  std::unique_ptr<ModeManagerCore> core_;
  std::unique_ptr<TrajectoryBuilder> builder_;
  size_t min_valid_points_{3};
  bool manage_nav2_{true};
  std::optional<bool> nav2_active_commanded_;
  Decision last_decision_;

  std::optional<Trajectory> latest_onroad_;
  std::optional<Trajectory> latest_offroad_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;

  rclcpp::Publisher<Trajectory>::SharedPtr output_publisher_;
  rclcpp::Publisher<TrajectoryModeState>::SharedPtr status_publisher_;
  rclcpp::Subscription<Trajectory>::SharedPtr onroad_subscription_;
  rclcpp::Subscription<Trajectory>::SharedPtr offroad_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Service<ChangeTrajectoryMode>::SharedPtr change_mode_service_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr nav2_lifecycle_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::TrajectoryModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
