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

#include "offroad_goal_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <cmath>
#include <string>

namespace autoware::nav2_offroad::rviz_plugin
{

OffroadGoalTool::OffroadGoalTool()
: rviz_default_plugins::tools::PoseTool(), topic_property_(nullptr)
{
  shortcut_key_ = 'o';
}

void OffroadGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Off-road Goal");
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/planning/offroad_goal", "The topic on which to publish the off-road goal.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  updateTopic();
}

void OffroadGoalTool::updateTopic()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS{1});
  clock_ = node->get_clock();
}

void OffroadGoalTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = context_->getFixedFrame().toStdString();
  goal.header.stamp = clock_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.z = std::sin(theta * 0.5);
  goal.pose.orientation.w = std::cos(theta * 0.5);
  publisher_->publish(goal);
}

}  // namespace autoware::nav2_offroad::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::nav2_offroad::rviz_plugin::OffroadGoalTool, rviz_common::Tool)
