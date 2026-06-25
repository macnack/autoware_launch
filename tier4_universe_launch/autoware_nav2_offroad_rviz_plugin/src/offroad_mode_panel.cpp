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

#include "offroad_mode_panel.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <std_srvs/srv/trigger.hpp>

#include <cstdio>
#include <memory>
#include <string>

namespace autoware::nav2_offroad::rviz_plugin
{
namespace
{
constexpr char kChangeModeService[] = "/nav2_offroad/mode_manager/change_mode";
constexpr char kStatusTopic[] = "/nav2_offroad/mode_manager/status";
constexpr char kDebugTopic[] = "/nav2_offroad/mode_manager/debug";
constexpr char kLifecycleService[] = "/lifecycle_manager_navigation/manage_nodes";
constexpr char kOffroadCancelTopic[] = "/planning/offroad_cancel";
}  // namespace

using TrajectoryModeState = autoware_nav2_offroad_msgs::msg::TrajectoryModeState;
using TrajectoryModeDebug = autoware_nav2_offroad_msgs::msg::TrajectoryModeDebug;
using ChangeTrajectoryMode = autoware_nav2_offroad_msgs::srv::ChangeTrajectoryMode;
using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
using ReturnHomeState = autoware_nav2_offroad_msgs::msg::ReturnHomeState;

OffroadModePanel::OffroadModePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  offroad_button_ = new QPushButton("Activate OFF-ROAD (Nav2)");
  onroad_button_ = new QPushButton("Activate ON-ROAD (Autoware)");
  restart_nav2_button_ = new QPushButton("Restart Nav2 stack");
  status_label_ = new QLabel("mode: (unknown)");
  guard_label_ = new QLabel("guards: (waiting for ~/debug)");
  guard_label_->setStyleSheet("font-family: monospace;");
  nav2_label_ = new QLabel("nav2: (idle)");

  auto * button_layout = new QHBoxLayout;
  button_layout->addWidget(offroad_button_);
  button_layout->addWidget(onroad_button_);

  // Return To Home group
  set_home_button_ = new QPushButton("Set Home");
  return_home_button_ = new QPushButton("Return Home");
  cancel_return_button_ = new QPushButton("Cancel");
  rth_status_label_ = new QLabel("RTH: (no status)");

  auto * rth_button_layout = new QHBoxLayout;
  rth_button_layout->addWidget(set_home_button_);
  rth_button_layout->addWidget(return_home_button_);
  rth_button_layout->addWidget(cancel_return_button_);

  auto * rth_group_layout = new QVBoxLayout;
  rth_group_layout->addLayout(rth_button_layout);
  rth_group_layout->addWidget(rth_status_label_);

  auto * rth_group = new QGroupBox("Return To Home");
  rth_group->setLayout(rth_group_layout);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(status_label_);
  layout->addWidget(guard_label_);
  layout->addLayout(button_layout);
  layout->addWidget(restart_nav2_button_);
  layout->addWidget(nav2_label_);
  layout->addWidget(rth_group);
  setLayout(layout);

  connect(offroad_button_, &QPushButton::clicked, this, &OffroadModePanel::onClickOffroad);
  connect(onroad_button_, &QPushButton::clicked, this, &OffroadModePanel::onClickOnroad);
  connect(restart_nav2_button_, &QPushButton::clicked, this, &OffroadModePanel::onRestartNav2);
  connect(set_home_button_, &QPushButton::clicked, this, &OffroadModePanel::onSetHome);
  connect(return_home_button_, &QPushButton::clicked, this, &OffroadModePanel::onReturnHomeClicked);
  connect(cancel_return_button_, &QPushButton::clicked, this, &OffroadModePanel::onCancelReturn);
}

void OffroadModePanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  auto node = rviz_ros_node_.lock()->get_raw_node();

  client_ = node->create_client<ChangeTrajectoryMode>(kChangeModeService);
  lifecycle_client_ = node->create_client<ManageLifecycleNodes>(kLifecycleService);
  offroad_cancel_pub_ =
    node->create_publisher<std_msgs::msg::Bool>(kOffroadCancelTopic, rclcpp::QoS{1});

  status_sub_ = node->create_subscription<TrajectoryModeState>(
    kStatusTopic, rclcpp::QoS{1}.transient_local(),
    std::bind(&OffroadModePanel::onStatus, this, std::placeholders::_1));
  debug_sub_ = node->create_subscription<TrajectoryModeDebug>(
    kDebugTopic, rclcpp::QoS{1},
    std::bind(&OffroadModePanel::onDebug, this, std::placeholders::_1));
  rth_status_sub_ = node->create_subscription<ReturnHomeState>(
    "/return_home/status", rclcpp::QoS(1),
    std::bind(&OffroadModePanel::onReturnHomeStatus, this, std::placeholders::_1));
}

void OffroadModePanel::onClickOffroad()
{
  requestMode(TrajectoryModeState::MODE_NAV2_OFFROAD);
}

void OffroadModePanel::onClickOnroad()
{
  // Cancel the off-road (Nav2) scenario in scenario_selector so the stack falls
  // back to Autoware lane-driving even when no new on-road route has been set
  // (e.g. the vehicle is off the lanelet map and cannot be routed).
  if (offroad_cancel_pub_) {
    std_msgs::msg::Bool cancel;
    cancel.data = true;
    offroad_cancel_pub_->publish(cancel);
  }
  requestMode(TrajectoryModeState::MODE_AW_PLANNING);
}

void OffroadModePanel::requestMode(const std::string & target_mode)
{
  if (!client_) {
    return;
  }
  if (!client_->service_is_ready()) {
    status_label_->setText(
      QString::fromStdString("mode: service unavailable (" + std::string(kChangeModeService) + ")"));
    return;
  }

  auto request = std::make_shared<ChangeTrajectoryMode::Request>();
  request->target_mode = target_mode;
  request->force = false;

  client_->async_send_request(
    request, [this](rclcpp::Client<ChangeTrajectoryMode>::SharedFuture future) {
      const auto & response = future.get();
      const std::string text = response->accepted
                                 ? "mode: -> " + response->current_mode
                                 : "mode: rejected (" + response->message + ")";
      status_label_->setText(QString::fromStdString(text));
    });
}

void OffroadModePanel::onStatus(TrajectoryModeState::ConstSharedPtr msg)
{
  std::string text = "mode: " + msg->current_mode;
  if (!msg->transition.empty() && msg->transition != TrajectoryModeState::TRANSITION_NONE) {
    text += "  (" + msg->transition + ")";
  }
  if (!msg->fault_reason.empty()) {
    text += "  fault: " + msg->fault_reason;
  }
  status_label_->setText(QString::fromStdString(text));
}

void OffroadModePanel::onDebug(TrajectoryModeDebug::ConstSharedPtr msg)
{
  char buf[320];
  std::snprintf(
    buf, sizeof(buf),
    "target: %s\n"
    "pos %.2f / %.2f m  %s\n"
    "yaw %.2f / %.2f rad  %s\n"
    "vel %.2f / %.2f m/s  %s\n"
    "onroad %s (%.2fs)   offroad %s (%.2fs)",
    msg->target_is_offroad ? "offroad" : "onroad", msg->position_gap_m, msg->max_position_gap_m,
    msg->position_gap_m <= msg->max_position_gap_m ? "OK" : "X", msg->yaw_gap_rad,
    msg->max_yaw_gap_rad, msg->yaw_gap_rad <= msg->max_yaw_gap_rad ? "OK" : "X",
    msg->velocity_gap_mps, msg->max_velocity_step_mps,
    msg->velocity_gap_mps <= msg->max_velocity_step_mps ? "OK" : "X",
    msg->onroad_usable ? "live" : "--", msg->onroad_age_s, msg->offroad_usable ? "live" : "--",
    msg->offroad_age_s);
  guard_label_->setText(QString::fromUtf8(buf));
}

void OffroadModePanel::onRestartNav2()
{
  if (!lifecycle_client_ || !lifecycle_client_->service_is_ready()) {
    nav2_label_->setText(
      QString::fromStdString("nav2: lifecycle service unavailable (" + std::string(kLifecycleService) + ")"));
    return;
  }

  // RESET then STARTUP so this recovers from any state (active, inactive, or an
  // aborted bringup) — a plain STARTUP fails if the nodes are already active.
  nav2_label_->setText("nav2: resetting...");
  auto reset_request = std::make_shared<ManageLifecycleNodes::Request>();
  reset_request->command = ManageLifecycleNodes::Request::RESET;
  lifecycle_client_->async_send_request(
    reset_request, [this](rclcpp::Client<ManageLifecycleNodes>::SharedFuture) {
      nav2_label_->setText("nav2: starting up...");
      auto startup_request = std::make_shared<ManageLifecycleNodes::Request>();
      startup_request->command = ManageLifecycleNodes::Request::STARTUP;
      lifecycle_client_->async_send_request(
        startup_request, [this](rclcpp::Client<ManageLifecycleNodes>::SharedFuture future) {
          const bool ok = future.get()->success;
          nav2_label_->setText(ok ? "nav2: active" : "nav2: startup FAILED (set initial pose first)");
        });
    });
}

void OffroadModePanel::callReturnHomeTrigger(const std::string & service)
{
  auto node = rviz_ros_node_.lock()->get_raw_node();
  auto client = node->create_client<std_srvs::srv::Trigger>(service);
  if (!client->wait_for_service(std::chrono::milliseconds(200))) {
    RCLCPP_WARN(node->get_logger(), "service %s unavailable", service.c_str());
    return;
  }
  client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

void OffroadModePanel::onSetHome() { callReturnHomeTrigger("/return_home/set_home"); }
void OffroadModePanel::onReturnHomeClicked() { callReturnHomeTrigger("/return_home/return_home"); }
void OffroadModePanel::onCancelReturn() { callReturnHomeTrigger("/return_home/cancel_return"); }

void OffroadModePanel::onReturnHomeStatus(ReturnHomeState::ConstSharedPtr msg)
{
  const char * rr[] = {"NONE", "IN_PROGRESS", "REACHED", "CANCELED", "FAILED"};
  rth_status_label_->setText(QString::fromStdString(
    std::string(msg->has_home ? "home set" : "no home") +
    (msg->returning ? " | RETURNING" : "") +
    " | " + QString::number(msg->distance_to_home_m, 'f', 1).toStdString() + " m | " +
    rr[msg->result < 5 ? msg->result : 0]));
}

}  // namespace autoware::nav2_offroad::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::nav2_offroad::rviz_plugin::OffroadModePanel, rviz_common::Panel)
