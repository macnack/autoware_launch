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

#include <gtest/gtest.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <optional>

namespace
{
double durationToSec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

geometry_msgs::msg::PoseStamped makePose(const double x, const double y)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

TEST(TrajectoryBuilder, CreatesStopTrajectoryWithThreeZeroSpeedPoints)
{
  autoware::nav2_offroad::TrajectoryBuilder builder(
    autoware::nav2_offroad::TrajectoryBuilderParams{});

  nav_msgs::msg::Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.orientation.w = 1.0;
  odometry.pose.pose.position.x = 10.0;
  odometry.pose.pose.position.y = 20.0;

  const auto trajectory = builder.createStopTrajectory(
    rclcpp::Time(123, 0, RCL_ROS_TIME), std::optional<nav_msgs::msg::Odometry>(odometry),
    std::nullopt);

  ASSERT_EQ(trajectory.points.size(), 3U);
  EXPECT_EQ(trajectory.header.frame_id, "map");
  for (const auto & point : trajectory.points) {
    EXPECT_DOUBLE_EQ(point.longitudinal_velocity_mps, 0.0);
    EXPECT_DOUBLE_EQ(point.lateral_velocity_mps, 0.0);
  }
}

TEST(TrajectoryBuilder, CreatesForwardTrajectoryAndStopsAtLastPoint)
{
  autoware::nav2_offroad::TrajectoryBuilderParams params;
  params.resample_interval_m = 0.5;
  params.cruise_speed_mps = 2.0;
  params.goal_taper_distance_m = 5.0;
  params.stop_trajectory_min_length_m = 0.2;
  params.min_trajectory_point_distance_m = 0.2;

  autoware::nav2_offroad::TrajectoryBuilder builder(params);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(2.0, 0.0));
  path.poses.push_back(makePose(4.0, 0.0));

  const auto trajectory = builder.createTrajectoryFromPath(
    rclcpp::Time(123, 0, RCL_ROS_TIME), path);

  ASSERT_GE(trajectory.points.size(), 3U);
  EXPECT_GT(trajectory.points.front().longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.points.back().longitudinal_velocity_mps, 0.0);

  double previous_time = -1.0;
  for (const auto & point : trajectory.points) {
    const double current_time = durationToSec(point.time_from_start);
    EXPECT_GE(current_time, previous_time);
    previous_time = current_time;
  }
}

TEST(TrajectoryBuilder, PinsFinalPointOrientationToGoalYaw)
{
  autoware::nav2_offroad::TrajectoryBuilder builder(
    autoware::nav2_offroad::TrajectoryBuilderParams{});

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));
  path.poses.push_back(makePose(2.0, 0.0));
  path.poses.push_back(makePose(4.0, 0.0));  // path runs along +x (tangent yaw ~ 0)

  const double goal_yaw = M_PI_2;
  const auto trajectory =
    builder.createTrajectoryFromPath(rclcpp::Time(123, 0, RCL_ROS_TIME), path, goal_yaw);
  ASSERT_GE(trajectory.points.size(), 2U);
  EXPECT_NEAR(yawOf(trajectory.points.back().pose.orientation), M_PI_2, 1e-3);

  // Without goal_yaw the last point keeps its path-tangent orientation (~0).
  const auto default_trajectory =
    builder.createTrajectoryFromPath(rclcpp::Time(123, 0, RCL_ROS_TIME), path);
  EXPECT_NEAR(yawOf(default_trajectory.points.back().pose.orientation), 0.0, 1e-3);
}

TEST(TrajectoryBuilder, BlendsHeadingToGoalOverFinalApproach)
{
  autoware::nav2_offroad::TrajectoryBuilderParams params;
  params.resample_interval_m = 0.5;
  params.min_trajectory_point_distance_m = 0.2;
  params.goal_heading_blend_distance_m = 4.0;
  autoware::nav2_offroad::TrajectoryBuilder builder(params);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (double x = 0.0; x <= 8.0 + 1e-9; x += 1.0) {
    path.poses.push_back(makePose(x, 0.0));  // path along +x (tangent yaw ~ 0)
  }

  const double goal_yaw = M_PI_2;
  const auto traj =
    builder.createTrajectoryFromPath(rclcpp::Time(123, 0, RCL_ROS_TIME), path, goal_yaw);
  ASSERT_GE(traj.points.size(), 5U);

  // Goal point reaches the goal heading.
  EXPECT_NEAR(yawOf(traj.points.back().pose.orientation), M_PI_2, 1e-2);
  // Far from the goal (start, beyond the blend distance) keeps the path tangent (~0).
  EXPECT_NEAR(yawOf(traj.points.front().pose.orientation), 0.0, 1e-2);

  // The second-to-last point is partway between tangent and goal heading: the
  // heading is blended over the approach rather than jumping only at the end.
  const double penultimate_yaw = yawOf(traj.points.at(traj.points.size() - 2).pose.orientation);
  EXPECT_GT(penultimate_yaw, 0.3);
  EXPECT_LT(penultimate_yaw, M_PI_2 - 0.01);

  // Heading is non-decreasing toward the goal across the final stretch.
  for (size_t i = traj.points.size() - 4; i + 1 < traj.points.size(); ++i) {
    EXPECT_LE(
      yawOf(traj.points.at(i).pose.orientation),
      yawOf(traj.points.at(i + 1).pose.orientation) + 1e-6);
  }
}

TEST(TrajectoryBuilder, ReturnsEmptyTrajectoryForDegeneratePath)
{
  autoware::nav2_offroad::TrajectoryBuilder builder(
    autoware::nav2_offroad::TrajectoryBuilderParams{});

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.push_back(makePose(0.0, 0.0));

  const auto trajectory = builder.createTrajectoryFromPath(
    rclcpp::Time(123, 0, RCL_ROS_TIME), path);

  EXPECT_TRUE(trajectory.points.empty());
}
