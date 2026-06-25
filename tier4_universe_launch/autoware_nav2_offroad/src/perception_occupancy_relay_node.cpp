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

#include "autoware_nav2_offroad/occupancy_grid_relay.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <string>

namespace autoware::nav2_offroad
{

// Bridges the Autoware perception occupancy grid (published continuously with
// volatile QoS) into the latched, transient-local input a Nav2 costmap
// StaticLayer expects. Lives alongside free_map_publisher; only one of the two
// drives the costmap, selected by the nav2_offroad launch.
class PerceptionOccupancyRelayNode : public rclcpp::Node
{
public:
  PerceptionOccupancyRelayNode() : Node("perception_occupancy_relay")
  {
    input_topic_ =
      declare_parameter<std::string>("input_topic", "/perception/occupancy_grid_map/map");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/nav2_offroad/costmap/occupancy_grid");
    unknown_as_free_ = declare_parameter<bool>("unknown_as_free", false);

    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS{1}.transient_local().reliable());
    subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, rclcpp::QoS{1},
      std::bind(&PerceptionOccupancyRelayNode::onGrid, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Relaying occupancy grid %s -> %s (unknown_as_free=%s)", input_topic_.c_str(),
      output_topic_.c_str(), unknown_as_free_ ? "true" : "false");
  }

private:
  void onGrid(nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (unknown_as_free_) {
      message->data = remapOccupancy(message->data, true);
    }
    publisher_->publish(*message);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool unknown_as_free_{false};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::PerceptionOccupancyRelayNode>());
  rclcpp::shutdown();
  return 0;
}
