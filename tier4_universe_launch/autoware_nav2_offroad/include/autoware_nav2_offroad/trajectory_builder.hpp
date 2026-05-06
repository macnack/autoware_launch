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

#ifndef AUTOWARE_NAV2_OFFROAD__TRAJECTORY_BUILDER_HPP_
#define AUTOWARE_NAV2_OFFROAD__TRAJECTORY_BUILDER_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::nav2_offroad
{

struct TrajectoryBuilderParams
{
  double resample_interval_m{0.5};
  double cruise_speed_mps{2.0};
  double goal_taper_distance_m{5.0};
  double stop_trajectory_min_length_m{0.5};
  double min_trajectory_point_distance_m{0.2};
};

double distance2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

class TrajectoryBuilder
{
public:
  explicit TrajectoryBuilder(TrajectoryBuilderParams params);

  [[nodiscard]] autoware_planning_msgs::msg::Trajectory createStopTrajectory(
    const rclcpp::Time & stamp, const std::optional<nav_msgs::msg::Odometry> & odometry,
    const std::optional<geometry_msgs::msg::PoseStamped> & goal) const;

  autoware_planning_msgs::msg::Trajectory createTrajectoryFromPath(
    const rclcpp::Time & stamp, const nav_msgs::msg::Path & path) const;

private:
  TrajectoryBuilderParams params_;
};

}  // namespace autoware::nav2_offroad

#endif  // AUTOWARE_NAV2_OFFROAD__TRAJECTORY_BUILDER_HPP_
