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

#ifndef OFFROAD_MODE_PANEL_HPP_
#define OFFROAD_MODE_PANEL_HPP_

#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <autoware_nav2_offroad_msgs/msg/return_home_state.hpp>
#include <autoware_nav2_offroad_msgs/msg/trajectory_mode_debug.hpp>
#include <autoware_nav2_offroad_msgs/msg/trajectory_mode_state.hpp>
#include <autoware_nav2_offroad_msgs/srv/change_trajectory_mode.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>

class QPushButton;
class QLabel;

namespace autoware::nav2_offroad::rviz_plugin
{
class OffroadModePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit OffroadModePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onClickOffroad();
  void onClickOnroad();
  void onRestartNav2();
  void onSetHome();
  void onReturnHomeClicked();
  void onCancelReturn();

private:
  void requestMode(const std::string & target_mode);
  void onStatus(autoware_nav2_offroad_msgs::msg::TrajectoryModeState::ConstSharedPtr msg);
  void onDebug(autoware_nav2_offroad_msgs::msg::TrajectoryModeDebug::ConstSharedPtr msg);
  void callReturnHomeTrigger(const std::string & service);
  void onReturnHomeStatus(autoware_nav2_offroad_msgs::msg::ReturnHomeState::ConstSharedPtr msg);

  QPushButton * offroad_button_;
  QPushButton * onroad_button_;
  QPushButton * restart_nav2_button_;
  QLabel * status_label_;
  QLabel * guard_label_;
  QLabel * nav2_label_;
  QPushButton * set_home_button_;
  QPushButton * return_home_button_;
  QPushButton * cancel_return_button_;
  QLabel * rth_status_label_;

  rclcpp::Client<autoware_nav2_offroad_msgs::srv::ChangeTrajectoryMode>::SharedPtr client_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr offroad_cancel_pub_;
  rclcpp::Subscription<autoware_nav2_offroad_msgs::msg::TrajectoryModeState>::SharedPtr status_sub_;
  rclcpp::Subscription<autoware_nav2_offroad_msgs::msg::TrajectoryModeDebug>::SharedPtr debug_sub_;
  rclcpp::Subscription<autoware_nav2_offroad_msgs::msg::ReturnHomeState>::SharedPtr rth_status_sub_;

protected:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};
}  // namespace autoware::nav2_offroad::rviz_plugin

#endif  // OFFROAD_MODE_PANEL_HPP_
