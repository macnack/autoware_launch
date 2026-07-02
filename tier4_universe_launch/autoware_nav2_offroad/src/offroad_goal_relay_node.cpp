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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>

namespace autoware::nav2_offroad
{
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// Relays the off-road goal topic into the NavigateToPose action bt_navigator
// expects (mppi mode). Latest goal wins: a new goal preempts the active one.
class OffroadGoalRelay : public rclcpp::Node
{
public:
  OffroadGoalRelay() : Node("offroad_goal_relay")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    pub_result_ = create_publisher<std_msgs::msg::UInt8>(
      "~/result", rclcpp::QoS(1).transient_local());
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/input/goal", rclcpp::QoS(1),
      std::bind(&OffroadGoalRelay::onGoal, this, std::placeholders::_1));
    sub_cancel_ = create_subscription<std_msgs::msg::Bool>(
      "~/input/cancel", rclcpp::QoS(1),
      std::bind(&OffroadGoalRelay::onCancel, this, std::placeholders::_1));
    publishResult(0);  // NONE — defined initial value for late transient_local subscribers
  }

private:
  void publishResult(uint8_t r)
  {
    std_msgs::msg::UInt8 m;
    m.data = r;
    pub_result_->publish(m);
  }

  void onGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    // NOTE: blocks the executor for up to 5 s while waiting for the server.
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server unavailable");
      publishResult(3);  // ABORTED
      return;
    }
    NavigateToPose::Goal goal;
    goal.pose = *msg;

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandle::SharedPtr gh) {
      if (!gh) {
        RCLCPP_ERROR(get_logger(), "goal rejected by navigate_to_pose server");
        publishResult(3);  // ABORTED
      }
    };
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000, "distance remaining: %.1f m",
          fb->distance_remaining);
      };
    opts.result_callback = [this](const GoalHandle::WrappedResult & r) {
      switch (r.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          publishResult(2);
          break;
        case rclcpp_action::ResultCode::CANCELED:
          publishResult(4);
          break;
        default:
          publishResult(3);  // ABORTED
          break;
      }
    };
    publishResult(1);  // ACTIVE (a new goal preempts: latest goal wins)
    client_->async_send_goal(goal, opts);
    RCLCPP_INFO(
      get_logger(), "relayed off-road goal (%.2f, %.2f) to navigate_to_pose",
      msg->pose.position.x, msg->pose.position.y);
  }

  void onCancel(std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    client_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "cancel requested for navigate_to_pose");
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_result_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cancel_;
};
}  // namespace autoware::nav2_offroad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::nav2_offroad::OffroadGoalRelay>());
  rclcpp::shutdown();
  return 0;
}
