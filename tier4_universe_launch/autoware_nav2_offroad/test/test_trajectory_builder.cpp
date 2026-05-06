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
