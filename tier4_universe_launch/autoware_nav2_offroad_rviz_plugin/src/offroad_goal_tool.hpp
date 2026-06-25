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

#ifndef OFFROAD_GOAL_TOOL_HPP_
#define OFFROAD_GOAL_TOOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rviz_common::properties
{
class StringProperty;
}  // namespace rviz_common::properties

namespace autoware::nav2_offroad::rviz_plugin
{
/// RViz tool (click-drag, like "2D Goal Pose") that publishes the pose to
/// /planning/offroad_goal so the Nav2 off-road planner / scenario_selector can
/// use it. Distinct from the on-road "2D Goal Pose" which goes to routing.
class OffroadGoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  OffroadGoalTool();
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  void publishFootprint(double x, double y, double theta);

  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::StringProperty * footprint_topic_property_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_publisher_;
  rclcpp::Clock::SharedPtr clock_;

  // Footprint extents relative to base_link (Autoware: base_link at rear axle).
  double front_{3.79};
  double rear_{-1.1};
  double left_{0.95};
  double right_{-0.95};
};
}  // namespace autoware::nav2_offroad::rviz_plugin

#endif  // OFFROAD_GOAL_TOOL_HPP_
