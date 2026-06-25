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

#include "autoware_nav2_offroad/return_home_core.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <autoware_nav2_offroad_msgs/msg/return_home_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>

namespace autoware::nav2_offroad
{
using StateMsg = autoware_nav2_offroad_msgs::msg::ReturnHomeState;

class ReturnHomeNode : public rclcpp::Node
{
public:
  ReturnHomeNode()
  : Node("return_home"),
    core_(ReturnHomeParams{
      declare_parameter<double>("reached_distance_m", 1.0),
      declare_parameter<double>("stopped_velocity_mps", 0.1)})
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/offroad_goal");
    cancel_topic_ = declare_parameter<std::string>("cancel_topic", "/planning/offroad_cancel");
    const double rate = declare_parameter<double>("publish_rate_hz", 10.0);
    return_timeout_s_ = declare_parameter<double>("return_timeout_s", 120.0);

    sub_ego_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematic_state", rclcpp::QoS(1),
      std::bind(&ReturnHomeNode::onEgo, this, std::placeholders::_1));
    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(1));
    pub_cancel_ = create_publisher<std_msgs::msg::Bool>(cancel_topic_, rclcpp::QoS(1));
    pub_status_ = create_publisher<StateMsg>("~/status", rclcpp::QoS(1));
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(1));

    srv_set_home_ = create_service<std_srvs::srv::Trigger>(
      "~/set_home", std::bind(&ReturnHomeNode::onSetHome, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_return_ = create_service<std_srvs::srv::Trigger>(
      "~/return_home", std::bind(&ReturnHomeNode::onReturnHome, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_cancel_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel_return", std::bind(&ReturnHomeNode::onCancel, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&ReturnHomeNode::onTimer, this));
  }

private:
  void onEgo(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    ego_valid_ = true;
    ego_.x = msg->pose.pose.position.x;
    ego_.y = msg->pose.pose.position.y;
    ego_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    ego_vel_ = msg->twist.twist.linear.x;
    last_ego_quat_ = msg->pose.pose.orientation;
  }

  void onSetHome(const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (!ego_valid_) { res->success = false; res->message = "no ego pose"; return; }
    core_.setHome(ego_);
    home_quat_ = last_ego_quat_;
    RCLCPP_INFO(get_logger(), "home set at (%.2f, %.2f)", ego_.x, ego_.y);
    res->success = true; res->message = "home set";
  }

  void onReturnHome(const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (!core_.hasHome()) {
      RCLCPP_WARN(get_logger(), "return_home rejected: no home set");
      res->success = false; res->message = "no home set"; return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = now();
    goal.pose.position.x = core_.home().x;
    goal.pose.position.y = core_.home().y;
    goal.pose.orientation = home_quat_;
    pub_goal_->publish(goal);
    returning_ = true;
    result_ = StateMsg::IN_PROGRESS;
    return_start_time_ = now();
    RCLCPP_INFO(get_logger(), "returning home -> %s", goal_topic_.c_str());
    res->success = true; res->message = "returning home";
  }

  void onCancel(const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (!returning_) { res->success = false; res->message = "not returning"; return; }
    std_msgs::msg::Bool b; b.data = true; pub_cancel_->publish(b);
    returning_ = false;
    result_ = StateMsg::CANCELED;
    RCLCPP_INFO(get_logger(), "return canceled");
    res->success = true; res->message = "canceled";
  }

  void onTimer()
  {
    if (returning_ && ego_valid_ && core_.isHomeReached(ego_, ego_vel_)) {
      returning_ = false;
      result_ = StateMsg::REACHED;
      RCLCPP_INFO(get_logger(), "home reached");
    } else if (returning_ && (now() - return_start_time_).seconds() > return_timeout_s_) {
      returning_ = false;
      result_ = StateMsg::FAILED;
      std_msgs::msg::Bool b; b.data = true; pub_cancel_->publish(b);
      RCLCPP_WARN(
        get_logger(), "return-home FAILED: timeout after %.0f s without reaching home",
        return_timeout_s_);
    }
    publishStatus();
    publishMarkers();
  }

  void publishStatus()
  {
    StateMsg s;
    s.header.stamp = now();
    s.has_home = core_.hasHome();
    s.home_pose.position.x = core_.home().x;
    s.home_pose.position.y = core_.home().y;
    s.home_pose.orientation = home_quat_;
    s.returning = returning_;
    s.distance_to_home_m = ego_valid_ ? core_.distanceToHome(ego_) : 0.0;
    s.result = result_;
    pub_status_->publish(s);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker home;
    home.header.frame_id = "map";
    home.header.stamp = now();
    home.ns = "home"; home.id = 0;
    home.type = visualization_msgs::msg::Marker::SPHERE;
    home.action = core_.hasHome() ? visualization_msgs::msg::Marker::ADD
                                  : visualization_msgs::msg::Marker::DELETE;
    home.pose.position.x = core_.home().x;
    home.pose.position.y = core_.home().y;
    home.scale.x = home.scale.y = home.scale.z = 0.8;
    home.color.r = 1.0; home.color.b = 1.0; home.color.a = 1.0;
    arr.markers.push_back(home);
    pub_markers_->publish(arr);
  }

  ReturnHomeCore core_;
  std::string goal_topic_, cancel_topic_;
  bool ego_valid_{false};
  Pose2d ego_{};
  double ego_vel_{0.0};
  geometry_msgs::msg::Quaternion last_ego_quat_{};
  geometry_msgs::msg::Quaternion home_quat_{};
  bool returning_{false};
  uint8_t result_{0};  // ReturnHomeState::NONE
  double return_timeout_s_{120.0};
  rclcpp::Time return_start_time_{};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ego_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_cancel_;
  rclcpp::Publisher<StateMsg>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_home_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_return_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cancel_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::ReturnHomeNode>());
  rclcpp::shutdown();
  return 0;
}
