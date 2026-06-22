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

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <memory>
#include <string>

namespace autoware::nav2_offroad::rviz_plugin
{
namespace
{
constexpr char kChangeModeService[] = "/trajectory_mode_manager/change_mode";
constexpr char kStatusTopic[] = "/trajectory_mode_manager/status";
}  // namespace

using TrajectoryModeState = autoware_nav2_offroad_msgs::msg::TrajectoryModeState;
using ChangeTrajectoryMode = autoware_nav2_offroad_msgs::srv::ChangeTrajectoryMode;

OffroadModePanel::OffroadModePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  offroad_button_ = new QPushButton("Activate OFF-ROAD (Nav2)");
  onroad_button_ = new QPushButton("Activate ON-ROAD (Autoware)");
  status_label_ = new QLabel("mode: (unknown)");

  auto * button_layout = new QHBoxLayout;
  button_layout->addWidget(offroad_button_);
  button_layout->addWidget(onroad_button_);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(status_label_);
  layout->addLayout(button_layout);
  setLayout(layout);

  connect(offroad_button_, &QPushButton::clicked, this, &OffroadModePanel::onClickOffroad);
  connect(onroad_button_, &QPushButton::clicked, this, &OffroadModePanel::onClickOnroad);
}

void OffroadModePanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();
  auto node = rviz_ros_node_.lock()->get_raw_node();

  client_ = node->create_client<ChangeTrajectoryMode>(kChangeModeService);

  status_sub_ = node->create_subscription<TrajectoryModeState>(
    kStatusTopic, rclcpp::QoS{1}.transient_local(),
    std::bind(&OffroadModePanel::onStatus, this, std::placeholders::_1));
}

void OffroadModePanel::onClickOffroad()
{
  requestMode(TrajectoryModeState::MODE_NAV2_OFFROAD);
}

void OffroadModePanel::onClickOnroad()
{
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

}  // namespace autoware::nav2_offroad::rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware::nav2_offroad::rviz_plugin::OffroadModePanel, rviz_common::Panel)
