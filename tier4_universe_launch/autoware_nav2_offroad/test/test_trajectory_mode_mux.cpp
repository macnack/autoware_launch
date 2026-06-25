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

#include "autoware_nav2_offroad/trajectory_mode_mux_node.hpp"

#include <gtest/gtest.h>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using Trajectory = autoware_planning_msgs::msg::Trajectory;

class TrajectoryMuxTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    mux_node_ = std::make_shared<TrajectoryModeMuxNode>();
    helper_node_ = std::make_shared<rclcpp::Node>("test_helper");

    executor_->add_node(mux_node_);
    executor_->add_node(helper_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    pub_onroad_ = helper_node_->create_publisher<Trajectory>(
      "input/onroad/trajectory", rclcpp::QoS{1});
    pub_offroad_ = helper_node_->create_publisher<Trajectory>(
      "input/offroad/trajectory", rclcpp::QoS{1});

    sub_output_ = helper_node_->create_subscription<Trajectory>(
      "output/trajectory", rclcpp::QoS{10},
      [this](Trajectory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_received_ = *msg;
        cv_.notify_one();
      });

    client_ = helper_node_->create_client<std_srvs::srv::SetBool>(
      "/trajectory_mode_mux/set_mode");

    waitForConnections();
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  // Block until both publishers have a matched subscriber on the mux node.
  void waitForConnections()
  {
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (
      (pub_onroad_->get_subscription_count() == 0 ||
       pub_offroad_->get_subscription_count() == 0) &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(pub_onroad_->get_subscription_count(), 0u) << "onroad sub not matched";
    ASSERT_GT(pub_offroad_->get_subscription_count(), 0u) << "offroad sub not matched";
    ASSERT_TRUE(client_->wait_for_service(2s)) << "set_mode service not available";
  }

  // Reset the last received message and return a fresh wait handle.
  void resetReceived()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_received_.reset();
  }

  // Wait up to `timeout` for a message on the output topic. Returns the message if received.
  std::optional<Trajectory> waitForMessage(std::chrono::milliseconds timeout = 1000ms)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    bool ok = cv_.wait_for(lock, timeout, [this]() { return last_received_.has_value(); });
    if (!ok) return std::nullopt;
    return last_received_;
  }

  // Call ~/set_mode synchronously. Returns the service success flag.
  bool callSetMode(bool offroad)
  {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = offroad;
    auto future = client_->async_send_request(req);
    if (future.wait_for(2s) != std::future_status::ready) return false;
    return future.get()->success;
  }

  static Trajectory makeTrajectory(const std::string & frame_id)
  {
    Trajectory traj;
    traj.header.frame_id = frame_id;
    return traj;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<TrajectoryModeMuxNode> mux_node_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  std::thread spin_thread_;

  rclcpp::Publisher<Trajectory>::SharedPtr pub_onroad_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_offroad_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_output_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Trajectory> last_received_;
};

TEST_F(TrajectoryMuxTest, OnRoadMode_ForwardsOnRoadTrajectory)
{
  pub_onroad_->publish(makeTrajectory("onroad_frame"));

  auto msg = waitForMessage();
  ASSERT_TRUE(msg.has_value()) << "no message received within timeout";
  EXPECT_EQ(msg->header.frame_id, "onroad_frame");
}

TEST_F(TrajectoryMuxTest, OnRoadMode_DropsOffroadTrajectory)
{
  pub_offroad_->publish(makeTrajectory("offroad_frame"));

  auto msg = waitForMessage(200ms);
  EXPECT_FALSE(msg.has_value()) << "offroad trajectory should be dropped in on-road mode";
}

TEST_F(TrajectoryMuxTest, SwitchToOffroad_ForwardsOffroadTrajectory)
{
  ASSERT_TRUE(callSetMode(true));

  pub_offroad_->publish(makeTrajectory("offroad_frame"));

  auto msg = waitForMessage();
  ASSERT_TRUE(msg.has_value()) << "no message received within timeout";
  EXPECT_EQ(msg->header.frame_id, "offroad_frame");
}

TEST_F(TrajectoryMuxTest, OffroadMode_DropsOnRoadTrajectory)
{
  ASSERT_TRUE(callSetMode(true));

  pub_onroad_->publish(makeTrajectory("onroad_frame"));

  auto msg = waitForMessage(200ms);
  EXPECT_FALSE(msg.has_value()) << "onroad trajectory should be dropped in offroad mode";
}

TEST_F(TrajectoryMuxTest, SwitchBackToOnRoad_ForwardsOnRoadTrajectory)
{
  ASSERT_TRUE(callSetMode(true));
  ASSERT_TRUE(callSetMode(false));

  pub_onroad_->publish(makeTrajectory("onroad_frame"));

  auto msg = waitForMessage();
  ASSERT_TRUE(msg.has_value()) << "no message received within timeout";
  EXPECT_EQ(msg->header.frame_id, "onroad_frame");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
