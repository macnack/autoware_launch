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
  footprint_topic_property_ = new rviz_common::properties::StringProperty(
    "Footprint Topic", "/planning/offroad_goal/footprint",
    "Topic for the vehicle-footprint marker drawn at the goal pose.", getPropertyContainer(),
    SLOT(updateTopic()), this);
  updateTopic();
}

void OffroadGoalTool::updateTopic()
{
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS{1});
  footprint_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>(
    footprint_topic_property_->getStdString(), rclcpp::QoS{1}.transient_local());
  clock_ = node->get_clock();

  // Read the vehicle footprint from the vehicle_info params (passed to every
  // node by the launch); fall back to sample_vehicle dimensions.
  const auto dim = [&node](const std::string & name, double def) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, def);
    }
    return node->get_parameter(name).as_double();
  };
  const double wheel_base = dim("wheel_base", 2.79);
  const double front_overhang = dim("front_overhang", 1.0);
  const double rear_overhang = dim("rear_overhang", 1.1);
  const double wheel_tread = dim("wheel_tread", 1.64);
  const double left_overhang = dim("left_overhang", 0.128);
  const double right_overhang = dim("right_overhang", 0.128);
  front_ = wheel_base + front_overhang;
  rear_ = -rear_overhang;
  left_ = wheel_tread * 0.5 + left_overhang;
  right_ = -(wheel_tread * 0.5 + right_overhang);
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

  publishFootprint(x, y, theta);
}

void OffroadGoalTool::publishFootprint(double x, double y, double theta)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  // Closed rectangle in base_link frame: front-left, front-right, rear-right, rear-left, close.
  const double corner_x[5] = {front_, front_, rear_, rear_, front_};
  const double corner_y[5] = {left_, right_, right_, left_, left_};

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = context_->getFixedFrame().toStdString();
  marker.header.stamp = clock_->now();
  marker.ns = "offroad_goal_footprint";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color.a = 1.0F;
  marker.color.g = 1.0F;
  marker.pose.orientation.w = 1.0;
  for (int i = 0; i < 5; ++i) {
    geometry_msgs::msg::Point p;
    p.x = x + corner_x[i] * c - corner_y[i] * s;
    p.y = y + corner_x[i] * s + corner_y[i] * c;
    p.z = 0.0;
    marker.points.push_back(p);
  }
  footprint_publisher_->publish(marker);
}

}  // namespace autoware::nav2_offroad::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::nav2_offroad::rviz_plugin::OffroadGoalTool, rviz_common::Tool)
