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

#include "autoware_nav2_offroad/trajectory_builder.hpp"
#include "autoware_nav2_offroad/trajectory_validator_core.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::nav2_offroad
{
using autoware_planning_msgs::msg::Trajectory;

class TrajectoryValidatorNode : public rclcpp::Node
{
public:
  TrajectoryValidatorNode() : Node("trajectory_validator"), diagnostics_(this)
  {
    ValidatorParams vp;
    vp.min_points = static_cast<std::size_t>(
      std::max(static_cast<int>(declare_parameter<int>("min_points", 2)), 2));
    vp.max_velocity_mps = declare_parameter<double>("max_velocity_mps", 5.0);
    vp.max_longitudinal_accel_mps2 = declare_parameter<double>("max_longitudinal_accel_mps2", 2.0);
    vp.max_lateral_accel_mps2 = declare_parameter<double>("max_lateral_accel_mps2", 2.0);
    vp.max_curvature_1pm = declare_parameter<double>("max_curvature_1pm", 1.0);
    vp.max_position_gap_m = declare_parameter<double>("max_position_gap_m", 2.0);
    vp.max_yaw_gap_rad = declare_parameter<double>("max_yaw_gap_rad", 0.5);
    vp.max_velocity_step_mps = declare_parameter<double>("max_velocity_step_mps", 1.0);
    core_ = std::make_unique<TrajectoryValidatorCore>(vp);

    builder_ = std::make_unique<TrajectoryBuilder>(TrajectoryBuilderParams{});

    output_publisher_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});
    trajectory_subscription_ = create_subscription<Trajectory>(
      "~/input/trajectory", rclcpp::QoS{1},
      std::bind(&TrajectoryValidatorNode::onTrajectory, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      std::bind(&TrajectoryValidatorNode::onOdometry, this, std::placeholders::_1));

    diagnostics_.setHardwareID("trajectory_validator");
    diagnostics_.add("validation", this, &TrajectoryValidatorNode::produceDiagnostics);
  }

private:
  EgoState toEgoState() const
  {
    EgoState ego;
    if (!latest_odometry_) {
      return ego;
    }
    const auto & o = *latest_odometry_;
    if (
      !std::isfinite(o.pose.pose.position.x) || !std::isfinite(o.pose.pose.position.y) ||
      !std::isfinite(o.twist.twist.linear.x)) {
      return ego;  // invalid ego (non-finite) -> treated as no usable odometry
    }
    ego.valid = true;
    ego.pose.x = o.pose.pose.position.x;
    ego.pose.y = o.pose.pose.position.y;
    ego.pose.yaw = tf2::getYaw(o.pose.pose.orientation);
    ego.velocity_mps = o.twist.twist.linear.x;
    return ego;
  }

  static std::vector<TrajPoint> digest(const Trajectory & msg)
  {
    std::vector<TrajPoint> out;
    out.reserve(msg.points.size());
    for (const auto & pt : msg.points) {
      TrajPoint tp;
      tp.x = pt.pose.position.x;
      tp.y = pt.pose.position.y;
      tp.yaw = tf2::getYaw(pt.pose.orientation);
      tp.velocity_mps = pt.longitudinal_velocity_mps;
      tp.acceleration_mps2 = pt.acceleration_mps2;
      out.push_back(tp);
    }
    return out;
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odometry_ = *msg; }

  void onTrajectory(const Trajectory::SharedPtr msg)
  {
    const EgoState ego = toEgoState();
    if (!ego.valid) {
      awaiting_odometry_ = true;
      diagnostics_.force_update();
      return;  // no safe output without a current pose
    }
    awaiting_odometry_ = false;
    last_result_ = core_->validate(digest(*msg), ego);
    if (last_result_.feasible) {
      output_publisher_->publish(*msg);
    } else {
      output_publisher_->publish(
        builder_->createStopTrajectory(now(), latest_odometry_, std::nullopt));
    }
    diagnostics_.force_update();
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (awaiting_odometry_) {
      stat.summary(DiagnosticStatus::WARN, "awaiting odometry");
      return;
    }
    if (last_result_.feasible) {
      stat.summary(DiagnosticStatus::OK, "trajectory feasible");
    } else {
      stat.summary(DiagnosticStatus::ERROR, "safe stop: " + last_result_.reason);
    }
    stat.add("failed_check", toString(last_result_.check));
    stat.add("point_index", static_cast<int>(last_result_.point_index));
    stat.add("worst_value", last_result_.worst_value);
    stat.add("limit", last_result_.limit);
  }

  std::unique_ptr<TrajectoryValidatorCore> core_;
  std::unique_ptr<TrajectoryBuilder> builder_;
  diagnostic_updater::Updater diagnostics_;
  bool awaiting_odometry_{true};
  ValidationResult last_result_{};
  std::optional<nav_msgs::msg::Odometry> latest_odometry_;

  rclcpp::Publisher<Trajectory>::SharedPtr output_publisher_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::TrajectoryValidatorNode>());
  rclcpp::shutdown();
  return 0;
}
