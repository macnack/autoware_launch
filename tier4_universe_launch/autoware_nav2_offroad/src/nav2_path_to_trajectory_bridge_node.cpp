// Copyright 2026 TIER IV, Inc.
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

#include "autoware_nav2_offroad/trajectory_builder.hpp"

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_adapi_v1_msgs/msg/motion_state.hpp>
#include <autoware_adapi_v1_msgs/srv/accept_start.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/smooth_path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

namespace
{
constexpr double kMinPublishRateHz = 1.0;
constexpr double kMinResampleIntervalM = 0.1;

builtin_interfaces::msg::Duration toDurationMsg(const double seconds)
{
  const double sanitized = std::max(0.0, seconds);
  const auto sec = static_cast<int32_t>(std::floor(sanitized));
  const auto nanosec = static_cast<uint32_t>((sanitized - static_cast<double>(sec)) * 1e9);

  builtin_interfaces::msg::Duration duration;
  duration.sec = sec;
  duration.nanosec = nanosec;
  return duration;
}
}  // namespace

class Nav2PathToTrajectoryBridgeNode : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using SmoothPath = nav2_msgs::action::SmoothPath;
  using ComputePathGoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using SmoothPathGoalHandle = rclcpp_action::ClientGoalHandle<SmoothPath>;

  Nav2PathToTrajectoryBridgeNode()
  : Node("nav2_path_to_trajectory_bridge")
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/mission_planning/goal");
    odometry_topic_ =
      declare_parameter<std::string>("odometry_topic", "/localization/kinematic_state");
    trajectory_topic_ =
      declare_parameter<std::string>("trajectory_topic", "/planning/trajectory");
    turn_indicators_topic_ = declare_parameter<std::string>(
      "turn_indicators_topic", "/planning/turn_indicators_cmd");
    hazard_lights_topic_ =
      declare_parameter<std::string>("hazard_lights_topic", "/planning/hazard_lights_cmd");
    gear_cmd_topic_ = declare_parameter<std::string>("gear_cmd_topic", "/planning/gear_cmd");
    engage_topic_ = declare_parameter<std::string>("engage_topic", "/autoware/engage");
    force_engage_ = declare_parameter<bool>("force_engage", true);
    auto_accept_start_ = declare_parameter<bool>("auto_accept_start", true);
    motion_state_topic_ =
      declare_parameter<std::string>("motion_state_topic", "/api/motion/state");
    accept_start_service_name_ =
      declare_parameter<std::string>("accept_start_service", "/api/motion/accept_start");

    planner_action_name_ =
      declare_parameter<std::string>("planner_action_name", "/compute_path_to_pose");
    smoother_action_name_ =
      declare_parameter<std::string>("smoother_action_name", "/smooth_path");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    smoother_id_ = declare_parameter<std::string>("smoother_id", "simple_smoother");

    publish_rate_hz_ =
      std::max(declare_parameter<double>("publish_rate_hz", 10.0), kMinPublishRateHz);
    const double resample_interval_m =
      std::max(declare_parameter<double>("resample_interval_m", 0.5), kMinResampleIntervalM);
    const double cruise_speed_mps =
      std::max(declare_parameter<double>("cruise_speed_mps", 2.0), 0.1);
    const double goal_taper_distance_m =
      std::max(declare_parameter<double>("goal_taper_distance_m", 5.0), 0.1);
    goal_reached_distance_m_ =
      std::max(declare_parameter<double>("goal_reached_distance_m", 0.8), 0.1);
    const double stop_trajectory_min_length_m =
      std::max(declare_parameter<double>("stop_trajectory_min_length_m", 0.1), 0.0);
    const double min_trajectory_point_distance_m =
      std::max(declare_parameter<double>("min_trajectory_point_distance_m", 0.2), 0.01);
    const double goal_heading_blend_distance_m =
      std::max(declare_parameter<double>("goal_heading_blend_distance_m", 4.0), 0.0);
    action_server_timeout_sec_ =
      std::max(declare_parameter<double>("action_server_timeout_sec", 1.0), 0.1);
    smoothing_timeout_sec_ =
      std::max(declare_parameter<double>("smoothing_timeout_sec", 0.5), 0.1);

    trajectory_builder_ = std::make_unique<autoware::nav2_offroad::TrajectoryBuilder>(
      autoware::nav2_offroad::TrajectoryBuilderParams{
        resample_interval_m,
        cruise_speed_mps,
        goal_taper_distance_m,
        stop_trajectory_min_length_m,
        min_trajectory_point_distance_m,
        goal_heading_blend_distance_m,
      });

    trajectory_publisher_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      trajectory_topic_, rclcpp::QoS{1});
    turn_indicator_publisher_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      turn_indicators_topic_, rclcpp::QoS{1}.transient_local());
    hazard_lights_publisher_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
      hazard_lights_topic_, rclcpp::QoS{1}.transient_local());
    gear_cmd_publisher_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      gear_cmd_topic_, rclcpp::QoS{1}.transient_local());
    engage_publisher_ = create_publisher<autoware_vehicle_msgs::msg::Engage>(
      engage_topic_, rclcpp::QoS{1});

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS{1},
      std::bind(&Nav2PathToTrajectoryBridgeNode::onGoal, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS{10},
      std::bind(&Nav2PathToTrajectoryBridgeNode::onOdometry, this, std::placeholders::_1));
    motion_state_subscription_ = create_subscription<autoware_adapi_v1_msgs::msg::MotionState>(
      motion_state_topic_, rclcpp::QoS{1}.transient_local(),
      std::bind(&Nav2PathToTrajectoryBridgeNode::onMotionState, this, std::placeholders::_1));

    planner_client_ = rclcpp_action::create_client<ComputePathToPose>(this, planner_action_name_);
    smoother_client_ = rclcpp_action::create_client<SmoothPath>(this, smoother_action_name_);
    accept_start_client_ =
      create_client<autoware_adapi_v1_msgs::srv::AcceptStart>(accept_start_service_name_);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(
      timer_period, std::bind(&Nav2PathToTrajectoryBridgeNode::onPublishTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Started Nav2 bridge: goal=%s odom=%s trajectory=%s planner_id=%s smoother_id=%s "
      "force_engage=%s engage_topic=%s",
      goal_topic_.c_str(), odometry_topic_.c_str(), trajectory_topic_.c_str(),
      planner_id_.c_str(), smoother_id_.c_str(), force_engage_ ? "true" : "false",
      engage_topic_.c_str());
  }

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    std::optional<nav_msgs::msg::Odometry> latest_odometry;
    uint64_t sequence = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_goal_ = *message;
      has_valid_path_ = false;
      ++goal_sequence_;
      sequence = goal_sequence_;
      planning_in_progress_ = false;
      accepted_start_for_goal_ = false;
      accept_start_attempted_for_goal_ = false;
      start_request_in_flight_ = false;
      engage_published_for_goal_ = false;
      latest_path_ = nav_msgs::msg::Path{};
      if (latest_odometry_) {
        latest_odometry = latest_odometry_;
      }
    }

    cancelActiveGoals();

    if (!latest_odometry) {
      RCLCPP_WARN(
        get_logger(),
        "Received a goal but odometry is unavailable. Planning starts once odometry arrives.");
      return;
    }

    startPlanning(sequence, *message, *latest_odometry);
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    std::optional<geometry_msgs::msg::PoseStamped> latest_goal;
    uint64_t sequence = 0;
    bool should_start_planning = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_odometry_ = *message;

      if (latest_goal_ && !planning_in_progress_ && !has_valid_path_) {
        latest_goal = latest_goal_;
        sequence = goal_sequence_;
        should_start_planning = true;
      }
    }

    if (should_start_planning && latest_goal) {
      startPlanning(sequence, *latest_goal, *message);
    }
  }

  void onPublishTimer()
  {
    std::optional<geometry_msgs::msg::PoseStamped> goal;
    std::optional<nav_msgs::msg::Odometry> odometry;
    nav_msgs::msg::Path path;
    bool has_valid_path = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = latest_goal_;
      odometry = latest_odometry_;
      path = latest_path_;
      has_valid_path = has_valid_path_;
    }

    autoware_planning_msgs::msg::Trajectory trajectory;
    const auto now_stamp = now();
    const bool reached_goal = goal && odometry &&
      autoware::nav2_offroad::distance2d(odometry->pose.pose.position, goal->pose.position) <=
        goal_reached_distance_m_;

    if (!has_valid_path || !goal || reached_goal) {
      trajectory = trajectory_builder_->createStopTrajectory(now_stamp, odometry, goal);
    } else {
      // Pin the final trajectory point to the goal heading (the planner plans to
      // a pose; preserve that orientation through to the controller).
      const double goal_yaw = tf2::getYaw(goal->pose.orientation);
      trajectory = trajectory_builder_->createTrajectoryFromPath(now_stamp, path, goal_yaw);
      if (trajectory.points.size() < 2) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Bridge generated an empty trajectory from path; publishing stop trajectory.");
        trajectory = trajectory_builder_->createStopTrajectory(now_stamp, odometry, goal);
      }
    }

    trajectory_publisher_->publish(trajectory);
    publishNeutralSignalCommands();
    publishGearCommand(has_valid_path && goal.has_value() && !reached_goal);
    publishForceEngage(goal.has_value() && has_valid_path && !reached_goal);
    maybeRequestAcceptStart(goal.has_value() && has_valid_path);
  }

  void onMotionState(const autoware_adapi_v1_msgs::msg::MotionState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_motion_state_ = *message;
  }

  void publishNeutralSignalCommands()
  {
    const auto now_stamp = now();

    autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicators;
    turn_indicators.stamp = now_stamp;
    turn_indicators.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
    turn_indicator_publisher_->publish(turn_indicators);

    autoware_vehicle_msgs::msg::HazardLightsCommand hazard_lights;
    hazard_lights.stamp = now_stamp;
    hazard_lights.command = autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
    hazard_lights_publisher_->publish(hazard_lights);
  }

  void publishForceEngage(const bool should_publish)
  {
    if (!force_engage_ || !should_publish) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (engage_published_for_goal_) {
        return;
      }
      engage_published_for_goal_ = true;
    }

    autoware_vehicle_msgs::msg::Engage engage;
    engage.stamp = now();
    engage.engage = true;
    engage_publisher_->publish(engage);
  }

  void publishGearCommand(const bool should_drive)
  {
    autoware_vehicle_msgs::msg::GearCommand gear_cmd;
    gear_cmd.stamp = now();
    gear_cmd.command = should_drive ? autoware_vehicle_msgs::msg::GearCommand::DRIVE
                                    : autoware_vehicle_msgs::msg::GearCommand::PARK;
    gear_cmd_publisher_->publish(gear_cmd);
  }

  void maybeRequestAcceptStart(const bool has_goal_and_path)
  {
    if (!auto_accept_start_ || !has_goal_and_path) {
      return;
    }

    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!latest_motion_state_) {
        return;
      }
      if (
        latest_motion_state_->state != autoware_adapi_v1_msgs::msg::MotionState::STARTING ||
        accepted_start_for_goal_ || accept_start_attempted_for_goal_ || start_request_in_flight_)
      {
        return;
      }
      sequence = goal_sequence_;
    }

    using AcceptStart = autoware_adapi_v1_msgs::srv::AcceptStart;
    if (!accept_start_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "AcceptStart service is not ready yet: %s", accept_start_service_name_.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sequence != goal_sequence_) {
        return;
      }
      if (accept_start_attempted_for_goal_ || start_request_in_flight_) {
        return;
      }
      start_request_in_flight_ = true;
      accept_start_attempted_for_goal_ = true;
    }

    auto request = std::make_shared<AcceptStart::Request>();
    accept_start_client_->async_send_request(
      request,
      [this, sequence](rclcpp::Client<AcceptStart>::SharedFuture future) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sequence != goal_sequence_) {
          return;
        }
        start_request_in_flight_ = false;
        const auto & response = future.get();
        if (response->status.success) {
          accepted_start_for_goal_ = true;
        } else {
          RCLCPP_WARN(
            get_logger(), "AcceptStart was rejected: code=%d message=%s", response->status.code,
            response->status.message.c_str());
        }
      });
  }

  void startPlanning(
    const uint64_t sequence, const geometry_msgs::msg::PoseStamped & goal,
    const nav_msgs::msg::Odometry & odometry)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sequence != goal_sequence_) {
        return;
      }
      if (planning_in_progress_) {
        return;
      }
      planning_in_progress_ = true;
    }

    if (!planner_client_->wait_for_action_server(std::chrono::duration<double>(action_server_timeout_sec_))) {
      onPlanningFailed(sequence, "planner action server is not available");
      return;
    }
    if (!smoother_client_->wait_for_action_server(std::chrono::duration<double>(action_server_timeout_sec_))) {
      onPlanningFailed(sequence, "smoother action server is not available");
      return;
    }

    ComputePathToPose::Goal compute_goal;
    compute_goal.goal = goal;
    compute_goal.start.header = odometry.header;
    compute_goal.start.pose = odometry.pose.pose;
    compute_goal.planner_id = planner_id_;
    compute_goal.use_start = true;

    auto goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions{};
    goal_options.goal_response_callback =
      [this, sequence](const std::shared_ptr<ComputePathGoalHandle> & goal_handle) {
        if (!goal_handle) {
          onPlanningFailed(sequence, "planner goal was rejected");
        }
      };
    goal_options.result_callback =
      [this, sequence](const ComputePathGoalHandle::WrappedResult & result) {
        onPlannerResult(sequence, result);
      };

    planner_client_->async_send_goal(compute_goal, goal_options);
  }

  void onPlannerResult(
    const uint64_t sequence, const ComputePathGoalHandle::WrappedResult & result)
  {
    if (sequence != goalSequence()) {
      return;
    }

    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->path.poses.size() < 2)
    {
      onPlanningFailed(sequence, "planner failed to produce a valid path");
      return;
    }

    sendSmootherGoal(sequence, result.result->path);
  }

  void sendSmootherGoal(const uint64_t sequence, const nav_msgs::msg::Path & raw_path)
  {
    SmoothPath::Goal smooth_goal;
    smooth_goal.path = raw_path;
    smooth_goal.smoother_id = smoother_id_;
    smooth_goal.check_for_collisions = false;
    smooth_goal.max_smoothing_duration = toDurationMsg(smoothing_timeout_sec_);

    auto goal_options = rclcpp_action::Client<SmoothPath>::SendGoalOptions{};
    goal_options.goal_response_callback =
      [this, sequence, raw_path](const std::shared_ptr<SmoothPathGoalHandle> & goal_handle) {
        if (!goal_handle) {
          setPathResult(sequence, raw_path, "smoother goal was rejected, using raw path");
        }
      };
    goal_options.result_callback =
      [this, sequence, raw_path](const SmoothPathGoalHandle::WrappedResult & result) {
        if (sequence != goalSequence()) {
          return;
        }

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
          result.result->path.poses.size() >= 2)
        {
          setPathResult(sequence, result.result->path, "");
          return;
        }

        setPathResult(sequence, raw_path, "smoother failed, using raw path");
      };

    smoother_client_->async_send_goal(smooth_goal, goal_options);
  }

  void setPathResult(
    const uint64_t sequence, const nav_msgs::msg::Path & path, const std::string & warning)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence != goal_sequence_) {
      return;
    }

    if (!warning.empty()) {
      RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }

    latest_path_ = path;
    has_valid_path_ = latest_path_.poses.size() >= 2;
    planning_in_progress_ = false;
  }

  void onPlanningFailed(const uint64_t sequence, const std::string & reason)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sequence != goal_sequence_) {
      return;
    }

    planning_in_progress_ = false;
    has_valid_path_ = false;
    latest_path_ = nav_msgs::msg::Path{};
    RCLCPP_WARN(get_logger(), "Off-road planning failed: %s", reason.c_str());
  }

  uint64_t goalSequence() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal_sequence_;
  }

  void cancelActiveGoals()
  {
    if (planner_client_->action_server_is_ready()) {
      planner_client_->async_cancel_all_goals();
    }
    if (smoother_client_->action_server_is_ready()) {
      smoother_client_->async_cancel_all_goals();
    }
  }

  std::string goal_topic_;
  std::string odometry_topic_;
  std::string trajectory_topic_;
  std::string turn_indicators_topic_;
  std::string hazard_lights_topic_;
  std::string gear_cmd_topic_;
  std::string engage_topic_;
  std::string planner_action_name_;
  std::string smoother_action_name_;
  std::string planner_id_;
  std::string smoother_id_;
  std::string motion_state_topic_;
  std::string accept_start_service_name_;

  double publish_rate_hz_{};
  double goal_reached_distance_m_{};
  double action_server_timeout_sec_{};
  double smoothing_timeout_sec_{};
  bool force_engage_{true};
  bool auto_accept_start_{true};
  std::unique_ptr<autoware::nav2_offroad::TrajectoryBuilder> trajectory_builder_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::MotionState>::SharedPtr
    motion_state_subscription_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicator_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_publisher_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_client_;
  rclcpp_action::Client<SmoothPath>::SharedPtr smoother_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::AcceptStart>::SharedPtr accept_start_client_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  mutable std::mutex mutex_;
  uint64_t goal_sequence_ = 0;
  bool planning_in_progress_ = false;
  bool has_valid_path_ = false;
  bool accepted_start_for_goal_ = false;
  bool accept_start_attempted_for_goal_ = false;
  bool start_request_in_flight_ = false;
  bool engage_published_for_goal_ = false;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;
  std::optional<autoware_adapi_v1_msgs::msg::MotionState> latest_motion_state_;
  nav_msgs::msg::Path latest_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav2PathToTrajectoryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
