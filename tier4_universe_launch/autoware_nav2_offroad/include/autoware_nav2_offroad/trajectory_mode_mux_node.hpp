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

#pragma once

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>

class TrajectoryModeMuxNode : public rclcpp::Node
{
public:
  explicit TrajectoryModeMuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("trajectory_mode_mux", options), offroad_mode_(false)
  {
    pub_trajectory_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "output/trajectory", rclcpp::QoS{1});

    sub_onroad_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "input/onroad/trajectory", rclcpp::QoS{1},
      [this](autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
        if (!offroad_mode_) pub_trajectory_->publish(*msg);
      });

    sub_offroad_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "input/offroad/trajectory", rclcpp::QoS{1},
      [this](autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
        if (offroad_mode_) pub_trajectory_->publish(*msg);
      });

    srv_set_mode_ = create_service<std_srvs::srv::SetBool>(
      "~/set_mode",
      [this](
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        offroad_mode_ = request->data;
        response->success = true;
        response->message = offroad_mode_ ? "switched to NAV2_OFFROAD" : "switched to ON_ROAD";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      });

    RCLCPP_INFO(get_logger(), "trajectory_mode_mux started in ON_ROAD mode");
  }

private:
  bool offroad_mode_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_onroad_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_offroad_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_mode_;
};
