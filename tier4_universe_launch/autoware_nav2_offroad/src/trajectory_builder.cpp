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

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace
{
constexpr double kStopTimeStepSec = 0.1;
constexpr size_t kControllerMinTrajectoryPoints = 3;

enum class MotionDirection
{
  kForward,
  kReverse,
};

struct TrajectoryPoseInfo
{
  geometry_msgs::msg::Pose pose;
  double heading_yaw{0.0};
  MotionDirection direction{MotionDirection::kForward};
  bool is_cusp{false};
  size_t source_index{0};
};

bool isNearlyZero(const double value)
{
  return std::fabs(value) < 1e-6;
}

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

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

void ensureValidOrientation(geometry_msgs::msg::Pose & pose)
{
  const double norm = std::sqrt(
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  if (norm < 1e-6) {
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
  }
}

geometry_msgs::msg::Pose interpolatePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to, const double ratio)
{
  geometry_msgs::msg::Pose interpolated;
  interpolated.position.x = from.position.x + (to.position.x - from.position.x) * ratio;
  interpolated.position.y = from.position.y + (to.position.y - from.position.y) * ratio;
  interpolated.position.z = from.position.z + (to.position.z - from.position.z) * ratio;
  interpolated.orientation = from.orientation;
  return interpolated;
}

double estimateYaw(
  const std::vector<geometry_msgs::msg::Pose> & poses, const size_t index, const double fallback_yaw)
{
  if (poses.size() < 2) {
    return fallback_yaw;
  }

  const auto & current = poses.at(index).position;
  const auto & reference =
    (index + 1 < poses.size()) ? poses.at(index + 1).position : poses.at(index - 1).position;
  const double dx =
    (index + 1 < poses.size()) ? (reference.x - current.x) : (current.x - reference.x);
  const double dy =
    (index + 1 < poses.size()) ? (reference.y - current.y) : (current.y - reference.y);

  if (isNearlyZero(dx) && isNearlyZero(dy)) {
    return fallback_yaw;
  }

  return std::atan2(dy, dx);
}

MotionDirection detectMotionDirection(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
{
  const double dx = to.position.x - from.position.x;
  const double dy = to.position.y - from.position.y;
  if (isNearlyZero(dx) && isNearlyZero(dy)) {
    return MotionDirection::kForward;
  }

  const double heading_yaw = tf2::getYaw(from.orientation);
  const double dot = dx * std::cos(heading_yaw) + dy * std::sin(heading_yaw);
  return dot < 0.0 ? MotionDirection::kReverse : MotionDirection::kForward;
}

std::vector<double> cumulativeDistances(const std::vector<geometry_msgs::msg::Pose> & poses)
{
  std::vector<double> distances(poses.size(), 0.0);
  for (size_t i = 1; i < poses.size(); ++i) {
    distances[i] = distances[i - 1] + autoware::nav2_offroad::distance2d(
      poses[i - 1].position, poses[i].position);
  }
  return distances;
}

std::vector<geometry_msgs::msg::Pose> resamplePath(
  const nav_msgs::msg::Path & path, const double interval_m)
{
  if (path.poses.size() < 2) {
    return {};
  }

  std::vector<geometry_msgs::msg::Pose> original;
  original.reserve(path.poses.size());
  for (const auto & pose_stamped : path.poses) {
    original.push_back(pose_stamped.pose);
  }

  const auto cumulative = cumulativeDistances(original);
  const double total_length = cumulative.back();
  if (total_length < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  std::vector<geometry_msgs::msg::Pose> sampled;
  sampled.reserve(static_cast<size_t>(std::ceil(total_length / interval_m)) + 2);

  size_t segment_index = 0;
  for (double query = 0.0; query < total_length; query += interval_m) {
    while (segment_index + 1 < cumulative.size() && cumulative[segment_index + 1] < query) {
      ++segment_index;
    }

    if (segment_index + 1 >= cumulative.size()) {
      break;
    }

    const double segment_length = cumulative[segment_index + 1] - cumulative[segment_index];
    const double ratio =
      segment_length > 1e-6 ? (query - cumulative[segment_index]) / segment_length : 0.0;
    sampled.push_back(interpolatePose(original[segment_index], original[segment_index + 1], ratio));
  }

  sampled.push_back(original.back());

  if (sampled.size() < 2) {
    return {};
  }
  return sampled;
}

std::vector<geometry_msgs::msg::Pose> removeCloseConsecutivePoses(
  const std::vector<geometry_msgs::msg::Pose> & poses, const double min_distance_m)
{
  if (poses.empty()) {
    return {};
  }

  std::vector<geometry_msgs::msg::Pose> filtered;
  filtered.reserve(poses.size());
  filtered.push_back(poses.front());

  for (size_t i = 1; i < poses.size(); ++i) {
    if (autoware::nav2_offroad::distance2d(filtered.back().position, poses[i].position) <
      min_distance_m)
    {
      continue;
    }
    filtered.push_back(poses[i]);
  }

  if (filtered.size() == 1 && poses.size() > 1) {
    filtered.push_back(poses.back());
  } else if (filtered.size() >= 2) {
    filtered.back() = poses.back();
  }

  return filtered;
}

void appendForwardPoint(std::vector<geometry_msgs::msg::Pose> & sampled, const double spacing_m)
{
  if (sampled.empty()) {
    return;
  }

  if (sampled.size() == 1) {
    auto extra_pose = sampled.back();
    const double yaw = tf2::getYaw(extra_pose.orientation);
    extra_pose.position.x += spacing_m * std::cos(yaw);
    extra_pose.position.y += spacing_m * std::sin(yaw);
    sampled.push_back(extra_pose);
    return;
  }

  const auto & previous = sampled[sampled.size() - 2];
  const auto & current = sampled.back();
  double dx = current.position.x - previous.position.x;
  double dy = current.position.y - previous.position.y;
  double norm = std::hypot(dx, dy);

  if (norm < 1e-6) {
    const double yaw = tf2::getYaw(current.orientation);
    dx = std::cos(yaw);
    dy = std::sin(yaw);
    norm = 1.0;
  }

  auto extra_pose = current;
  extra_pose.position.x += spacing_m * (dx / norm);
  extra_pose.position.y += spacing_m * (dy / norm);
  sampled.push_back(extra_pose);
}

void ensureMinimumSampleCount(
  std::vector<geometry_msgs::msg::Pose> & sampled, const size_t min_points,
  const double min_spacing_m)
{
  if (sampled.empty()) {
    return;
  }

  while (sampled.size() < min_points) {
    appendForwardPoint(sampled, min_spacing_m);
  }
}

autoware_planning_msgs::msg::TrajectoryPoint createZeroSpeedPoint(
  const geometry_msgs::msg::Pose & pose, const double time_from_start_sec)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.time_from_start = toDurationMsg(time_from_start_sec);
  point.pose = pose;
  point.longitudinal_velocity_mps = 0.0;
  point.lateral_velocity_mps = 0.0;
  point.acceleration_mps2 = 0.0;
  point.heading_rate_rps = 0.0;
  point.front_wheel_angle_rad = 0.0;
  point.rear_wheel_angle_rad = 0.0;
  return point;
}

double blendPointYaw(
  const double base_yaw, const double remaining_distance,
  const std::optional<double> & goal_yaw,
  const autoware::nav2_offroad::TrajectoryBuilderParams & params)
{
  if (!goal_yaw || params.goal_heading_blend_distance_m <= 1e-6) {
    return base_yaw;
  }

  const double ratio =
    std::clamp(remaining_distance / params.goal_heading_blend_distance_m, 0.0, 1.0);
  return *goal_yaw + ratio * normalizeAngle(base_yaw - *goal_yaw);
}

autoware_planning_msgs::msg::Trajectory createForwardOnlyTrajectory(
  const rclcpp::Time & stamp, const nav_msgs::msg::Path & path,
  const std::vector<geometry_msgs::msg::Pose> & sampled_for_output,
  const autoware::nav2_offroad::TrajectoryBuilderParams & params, const std::optional<double> & goal_yaw)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;
  trajectory.header.stamp = stamp;

  const auto cumulative = cumulativeDistances(sampled_for_output);
  const double total_length = cumulative.back();

  trajectory.points.reserve(sampled_for_output.size());

  double elapsed_sec = 0.0;
  double previous_velocity = 0.0;

  for (size_t i = 0; i < sampled_for_output.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = sampled_for_output[i];

    ensureValidOrientation(point.pose);
    const double fallback_yaw = tf2::getYaw(point.pose.orientation);
    const double tangent_yaw = estimateYaw(sampled_for_output, i, fallback_yaw);

    const double remaining = total_length - cumulative[i];
    const double point_yaw =
      blendPointYaw(tangent_yaw, remaining, goal_yaw, params);
    point.pose.orientation = createQuaternionFromYaw(point_yaw);

    double velocity =
      params.cruise_speed_mps * std::clamp(remaining / params.goal_taper_distance_m, 0.0, 1.0);
    if (i == sampled_for_output.size() - 1) {
      velocity = 0.0;
    }

    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lateral_velocity_mps = 0.0F;
    point.acceleration_mps2 = 0.0F;
    point.heading_rate_rps = 0.0F;
    point.front_wheel_angle_rad = 0.0F;
    point.rear_wheel_angle_rad = 0.0F;

    if (i > 0) {
      const double segment = cumulative[i] - cumulative[i - 1];
      const double average_velocity = std::max(0.1, 0.5 * (previous_velocity + velocity));
      elapsed_sec += segment / average_velocity;
    }
    point.time_from_start = toDurationMsg(elapsed_sec);

    previous_velocity = velocity;
    trajectory.points.push_back(point);
  }

  if (goal_yaw && !trajectory.points.empty()) {
    trajectory.points.back().pose.orientation = createQuaternionFromYaw(*goal_yaw);
  }

  return trajectory;
}

std::vector<TrajectoryPoseInfo> buildReverseAwarePoseInfos(
  const std::vector<geometry_msgs::msg::Pose> & sampled_for_output)
{
  std::vector<TrajectoryPoseInfo> pose_infos;
  if (sampled_for_output.size() < 2) {
    return pose_infos;
  }

  // Validate orientations once up front so direction detection here is consistent
  // with the has_reverse_segment pre-scan and the per-pose heading (all read the
  // pose orientation via tf2::getYaw); a degenerate quaternion must not make the
  // two disagree.
  std::vector<geometry_msgs::msg::Pose> poses = sampled_for_output;
  for (auto & pose : poses) {
    ensureValidOrientation(pose);
  }

  std::vector<MotionDirection> segment_directions;
  segment_directions.reserve(poses.size() - 1);
  for (size_t i = 0; i + 1 < poses.size(); ++i) {
    segment_directions.push_back(detectMotionDirection(poses[i], poses[i + 1]));
  }

  pose_infos.reserve(poses.size());
  for (size_t i = 0; i < poses.size(); ++i) {
    const double heading_yaw = tf2::getYaw(poses[i].orientation);
    const MotionDirection direction =
      (i + 1 < poses.size()) ? segment_directions[i] : segment_directions.back();
    const bool is_direction_switch =
      i > 0 && i + 1 < poses.size() && segment_directions[i] != segment_directions[i - 1];

    pose_infos.push_back(
      TrajectoryPoseInfo{poses[i], heading_yaw, direction, is_direction_switch, i});
  }

  return pose_infos;
}
}  // namespace

namespace autoware::nav2_offroad
{

double distance2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

TrajectoryBuilder::TrajectoryBuilder(TrajectoryBuilderParams params)
: params_(std::move(params))
{
}

autoware_planning_msgs::msg::Trajectory TrajectoryBuilder::createStopTrajectory(
  const rclcpp::Time & stamp, const std::optional<nav_msgs::msg::Odometry> & odometry,
  const std::optional<geometry_msgs::msg::PoseStamped> & goal) const
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";

  geometry_msgs::msg::Pose base_pose;
  base_pose.orientation.w = 1.0;
  if (odometry) {
    base_pose = odometry->pose.pose;
    trajectory.header.frame_id = odometry->header.frame_id.empty() ? "map" : odometry->header.frame_id;
  } else if (goal) {
    base_pose = goal->pose;
    trajectory.header.frame_id = goal->header.frame_id.empty() ? "map" : goal->header.frame_id;
  }

  ensureValidOrientation(base_pose);

  auto second_pose = base_pose;
  const double yaw = tf2::getYaw(base_pose.orientation);
  second_pose.position.x += params_.stop_trajectory_min_length_m * std::cos(yaw);
  second_pose.position.y += params_.stop_trajectory_min_length_m * std::sin(yaw);

  auto third_pose = second_pose;
  third_pose.position.x += params_.stop_trajectory_min_length_m * std::cos(yaw);
  third_pose.position.y += params_.stop_trajectory_min_length_m * std::sin(yaw);

  trajectory.points.push_back(createZeroSpeedPoint(base_pose, 0.0));
  trajectory.points.push_back(createZeroSpeedPoint(second_pose, kStopTimeStepSec));
  trajectory.points.push_back(createZeroSpeedPoint(third_pose, 2.0 * kStopTimeStepSec));
  return trajectory;
}

autoware_planning_msgs::msg::Trajectory TrajectoryBuilder::createTrajectoryFromPath(
  const rclcpp::Time & stamp, const nav_msgs::msg::Path & path,
  const std::optional<double> & goal_yaw) const
{
  const auto sampled = resamplePath(path, params_.resample_interval_m);
  if (sampled.size() < 2) {
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = path.header;
    trajectory.header.stamp = stamp;
    return trajectory;
  }

  auto sampled_for_output =
    removeCloseConsecutivePoses(sampled, params_.min_trajectory_point_distance_m);
  if (sampled_for_output.size() < 2) {
    autoware_planning_msgs::msg::Trajectory trajectory;
    trajectory.header = path.header;
    trajectory.header.stamp = stamp;
    return trajectory;
  }

  ensureMinimumSampleCount(
    sampled_for_output, kControllerMinTrajectoryPoints, params_.min_trajectory_point_distance_m);

  bool has_reverse_segment = false;
  for (size_t i = 0; i + 1 < sampled_for_output.size(); ++i) {
    auto pose = sampled_for_output[i];
    ensureValidOrientation(pose);
    if (detectMotionDirection(pose, sampled_for_output[i + 1]) == MotionDirection::kReverse) {
      has_reverse_segment = true;
      break;
    }
  }

  if (!has_reverse_segment) {
    return createForwardOnlyTrajectory(stamp, path, sampled_for_output, params_, goal_yaw);
  }

  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header = path.header;
  trajectory.header.stamp = stamp;

  const auto cumulative = cumulativeDistances(sampled_for_output);
  const double total_length = cumulative.back();
  const auto pose_infos = buildReverseAwarePoseInfos(sampled_for_output);

  trajectory.points.reserve(pose_infos.size());

  // Arc-length positions of the cusps (forward<->reverse switches). Velocity must
  // taper to zero approaching a cusp and ramp back up after it; otherwise the
  // profile would jump cruise -> 0 -> cruise at the cusp, which the controller
  // cannot track (the vehicle cannot stop instantaneously to reverse).
  std::vector<double> cusp_arc_positions;
  for (const auto & info : pose_infos) {
    if (info.is_cusp) {
      cusp_arc_positions.push_back(cumulative[info.source_index]);
    }
  }

  double elapsed_sec = 0.0;
  double previous_velocity = 0.0;
  geometry_msgs::msg::Point previous_position = pose_infos.front().pose.position;

  for (size_t i = 0; i < pose_infos.size(); ++i) {
    const auto & pose_info = pose_infos[i];
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = pose_info.pose;

    const double remaining = total_length - cumulative[pose_info.source_index];
    const double point_yaw =
      blendPointYaw(pose_info.heading_yaw, remaining, goal_yaw, params_);
    point.pose.orientation = createQuaternionFromYaw(point_yaw);

    double velocity = 0.0;
    if (!pose_info.is_cusp && i + 1 < pose_infos.size()) {
      // Taper by distance to the goal AND to the nearest cusp, whichever is closer.
      const double arc = cumulative[pose_info.source_index];
      double taper_distance = remaining;
      for (const double cusp_arc : cusp_arc_positions) {
        taper_distance = std::min(taper_distance, std::fabs(arc - cusp_arc));
      }
      const double velocity_magnitude =
        params_.cruise_speed_mps *
        std::clamp(taper_distance / params_.goal_taper_distance_m, 0.0, 1.0);
      velocity =
        pose_info.direction == MotionDirection::kReverse ? -velocity_magnitude : velocity_magnitude;
    }

    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lateral_velocity_mps = 0.0F;
    point.acceleration_mps2 = 0.0F;
    point.heading_rate_rps = 0.0F;
    point.front_wheel_angle_rad = 0.0F;
    point.rear_wheel_angle_rad = 0.0F;

    if (i > 0) {
      const double segment = distance2d(previous_position, point.pose.position);
      const double average_velocity =
        std::max(0.1, 0.5 * (std::fabs(previous_velocity) + std::fabs(velocity)));
      elapsed_sec += segment / average_velocity;
    }
    point.time_from_start = toDurationMsg(elapsed_sec);

    previous_velocity = velocity;
    previous_position = point.pose.position;
    trajectory.points.push_back(point);
  }

  if (goal_yaw && !trajectory.points.empty()) {
    trajectory.points.back().pose.orientation = createQuaternionFromYaw(*goal_yaw);
  }

  return trajectory;
}

}  // namespace autoware::nav2_offroad
