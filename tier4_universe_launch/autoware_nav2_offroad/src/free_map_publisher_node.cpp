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

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
constexpr double kDefaultPublishRateHz = 1.0;
constexpr double kMinResolution = 1e-3;
constexpr double kMinDimension = 1.0;

uint32_t toCellCount(const double length_m, const double resolution)
{
  // Use ceil to guarantee requested map bounds are covered.
  return static_cast<uint32_t>(std::max(1.0, std::ceil(length_m / resolution)));
}
}  // namespace

class FreeMapPublisherNode : public rclcpp::Node
{
public:
  FreeMapPublisherNode()
  : Node("free_map_publisher")
  {
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/perception/occupancy_grid_map/map");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    resolution_ = std::max(declare_parameter<double>("resolution", 0.5), kMinResolution);
    width_m_ = std::max(declare_parameter<double>("width_m", 400.0), kMinDimension);
    height_m_ = std::max(declare_parameter<double>("height_m", 400.0), kMinDimension);
    center_x_ = declare_parameter<double>("center_x", 0.0);
    center_y_ = declare_parameter<double>("center_y", 0.0);
    auto_center_from_odometry_ =
      declare_parameter<bool>("auto_center_from_odometry", true);
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/localization/kinematic_state");
    const double publish_rate_hz =
      std::max(declare_parameter<double>("publish_rate_hz", kDefaultPublishRateHz), 0.1);

    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS{1}.transient_local().reliable());

    rebuildMapTemplate(center_x_, center_y_);

    if (auto_center_from_odometry_) {
      centered_from_odometry_ = false;
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, rclcpp::QoS{10},
        std::bind(&FreeMapPublisherNode::onOdometry, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = create_wall_timer(period, std::bind(&FreeMapPublisherNode::publishMap, this));

    publishMap();
    RCLCPP_INFO(
      get_logger(),
      "Publishing free occupancy map on %s (%u x %u, %.3f m/cell). auto_center_from_odometry=%s",
      output_topic_.c_str(), map_template_.info.width, map_template_.info.height, resolution_,
      auto_center_from_odometry_ ? "true" : "false");
  }

private:
  void rebuildMapTemplate(const double center_x, const double center_y)
  {
    map_template_.info.resolution = resolution_;
    map_template_.info.width = toCellCount(width_m_, resolution_);
    map_template_.info.height = toCellCount(height_m_, resolution_);
    map_template_.info.origin.position.x = center_x - width_m_ * 0.5;
    map_template_.info.origin.position.y = center_y - height_m_ * 0.5;
    map_template_.info.origin.position.z = 0.0;
    map_template_.info.origin.orientation.x = 0.0;
    map_template_.info.origin.orientation.y = 0.0;
    map_template_.info.origin.orientation.z = 0.0;
    map_template_.info.origin.orientation.w = 1.0;
    map_template_.data.assign(
      static_cast<size_t>(map_template_.info.width) *
        static_cast<size_t>(map_template_.info.height),
      0);
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (centered_from_odometry_) {
      return;
    }

    if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odometry frame '%s' does not match map frame '%s'; waiting for matching odometry frame.",
        message->header.frame_id.c_str(), frame_id_.c_str());
      return;
    }

    center_x_ = message->pose.pose.position.x;
    center_y_ = message->pose.pose.position.y;
    rebuildMapTemplate(center_x_, center_y_);
    centered_from_odometry_ = true;
    publishMap();

    RCLCPP_INFO(
      get_logger(), "Centered free occupancy map from odometry at (%.3f, %.3f)", center_x_,
      center_y_);
  }

  void publishMap()
  {
    auto message = map_template_;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    publisher_->publish(message);
  }

  std::string output_topic_;
  std::string frame_id_;
  double resolution_{};
  double width_m_{};
  double height_m_{};
  double center_x_{};
  double center_y_{};
  bool auto_center_from_odometry_{false};
  bool centered_from_odometry_{true};
  std::string odometry_topic_;

  nav_msgs::msg::OccupancyGrid map_template_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FreeMapPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
