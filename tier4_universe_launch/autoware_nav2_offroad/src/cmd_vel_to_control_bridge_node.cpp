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

#include "autoware_nav2_offroad/cmd_vel_to_control.hpp"
#include "autoware_nav2_offroad/gear_arbiter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <cstdint>
#include <memory>

namespace autoware::nav2_offroad
{
class CmdVelToControlBridge : public rclcpp::Node
{
public:
  CmdVelToControlBridge() : Node("cmd_vel_to_control_bridge")
  {
    params_.wheelbase_m = declare_parameter<double>("wheelbase_m", 2.7);
    params_.max_steer_rad = declare_parameter<double>("max_steer_rad", 0.6);
    params_.min_speed_for_steer_mps = declare_parameter<double>("min_speed_for_steer_mps", 0.1);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);

    enabled_ = declare_parameter<bool>("initial_enabled", false);
    if (enabled_) {
      RCLCPP_INFO(get_logger(), "bridge starts ENABLED (initial_enabled=true)");
    }

    // Acceleration-driven vehicle interfaces (simple_planning_simulator ACC_GEARED,
    // most real interfaces) integrate Control.longitudinal.acceleration and ignore
    // the velocity setpoint — publishing velocity-only commands never moves the
    // vehicle. Track the commanded velocity with a P-law on measured speed.
    accel_gain_ = declare_parameter<double>("accel_gain", 1.5);
    accel_limit_mps2_ = declare_parameter<double>("accel_limit_mps2", 3.0);
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematic_state", rclcpp::QoS(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        vehicle_speed_mps_ = msg->twist.twist.linear.x;
      });

    enable_reverse_ = declare_parameter<bool>("enable_reverse", false);
    if (enable_reverse_) {
      const double stop_thr = declare_parameter<double>("gear_stop_threshold_mps", 0.1);
      const double deadband = declare_parameter<double>("gear_deadband_mps", 0.05);
      arbiter_ = std::make_unique<GearArbiter>(stop_thr, deadband);
      RCLCPP_INFO(get_logger(), "reverse ENABLED (stop-and-shift gear sequencing)");
    }

    pub_ctrl_ = create_publisher<autoware_control_msgs::msg::Control>(
      "~/output/control_cmd", rclcpp::QoS(1));
    pub_gear_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
      "~/output/gear_cmd", rclcpp::QoS(1));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "~/input/cmd_vel", rclcpp::QoS(1),
      std::bind(&CmdVelToControlBridge::onCmdVel, this, std::placeholders::_1));
    srv_enable_ = create_service<std_srvs::srv::SetBool>(
      "~/enable", std::bind(&CmdVelToControlBridge::onEnable, this,
        std::placeholders::_1, std::placeholders::_2));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&CmdVelToControlBridge::onWatchdog, this));
  }

private:
  void onEnable(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res)
  {
    enabled_ = req->data;
    if (!enabled_) last_steer_ = 0.0;
    // Reset the staleness clock on enable so the watchdog doesn't fire immediately.
    last_cmd_vel_time_ = now();
    res->success = true;
    RCLCPP_INFO(get_logger(), "bridge %s", enabled_ ? "ENABLED" : "DISABLED");
  }

  void onCmdVel(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    if (!enabled_) return;
    last_cmd_vel_time_ = now();
    double cmd_v = msg->linear.x;
    uint8_t gear_cmd = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    if (enable_reverse_) {
      const auto arb = arbiter_->update(cmd_v, vehicle_speed_mps_);
      cmd_v = arb.velocity_mps;
      gear_cmd = arb.gear == Gear::REVERSE
        ? autoware_vehicle_msgs::msg::GearCommand::REVERSE
        : autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    }
    last_gear_reverse_ = gear_cmd == autoware_vehicle_msgs::msg::GearCommand::REVERSE;
    const auto c = twistToControl(cmd_v, msg->angular.z, params_, last_steer_);
    last_steer_ = c.steering_tire_angle_rad;
    autoware_control_msgs::msg::Control ctrl;
    ctrl.stamp = now();
    ctrl.longitudinal.velocity = static_cast<float>(c.velocity_mps);
    ctrl.longitudinal.acceleration = static_cast<float>(computeAccelCommand(
      c.velocity_mps, vehicle_speed_mps_, last_gear_reverse_, accel_gain_, accel_limit_mps2_));
    ctrl.lateral.steering_tire_angle = static_cast<float>(c.steering_tire_angle_rad);
    pub_ctrl_->publish(ctrl);
    autoware_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = ctrl.stamp;
    gear.command = gear_cmd;
    pub_gear_->publish(gear);
  }

  void onWatchdog()
  {
    if (!enabled_) return;
    const double age_s = (now() - last_cmd_vel_time_).seconds();
    if (age_s > cmd_vel_timeout_s_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "cmd_vel stale (%.2f s > %.2f s); holding stop", age_s, cmd_vel_timeout_s_);
      autoware_control_msgs::msg::Control ctrl;
      ctrl.stamp = now();
      ctrl.longitudinal.velocity = 0.0f;
      // Actively brake to zero — acceleration-driven interfaces ignore the
      // velocity field, so a 0-velocity/0-acceleration command coasts forever.
      ctrl.longitudinal.acceleration = static_cast<float>(computeAccelCommand(
        0.0, vehicle_speed_mps_, last_gear_reverse_, accel_gain_, accel_limit_mps2_));
      ctrl.lateral.steering_tire_angle = static_cast<float>(last_steer_);
      pub_ctrl_->publish(ctrl);
    }
  }

  BicycleParams params_;
  bool enabled_{false};
  double last_steer_{0.0};
  double cmd_vel_timeout_s_{0.5};
  bool enable_reverse_{false};
  bool last_gear_reverse_{false};
  double vehicle_speed_mps_{0.0};
  double accel_gain_{1.5};
  double accel_limit_mps2_{3.0};
  std::unique_ptr<GearArbiter> arbiter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_ctrl_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};
}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::CmdVelToControlBridge>());
  rclcpp::shutdown();
  return 0;
}
