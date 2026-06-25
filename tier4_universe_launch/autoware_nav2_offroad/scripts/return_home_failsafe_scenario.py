#!/usr/bin/env python3
# Copyright 2026 Maciej Krupka maciej.krupka@put.poznan.pl
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Return-To-Home failsafe scenario + final error measurement.

Scenario flow:
  1. Optionally set an initial pose (--init-x / --init-y).
  2. Wait for an odometry message to arrive on /localization/kinematic_state.
  3. Record the current ego position as "home" and call /return_home/set_home.
  4. Drive the vehicle out by publishing a goal on /planning/offroad_goal at
     (ego + dx, ego + dy) — simulates the vehicle leaving home (--dx / --dy).
  5. Wait --drive-timeout seconds for the vehicle to move out.
  6. Call /return_home/return_home to trigger the failsafe.
  7. Poll /return_home/status until result == REACHED (2) or --timeout expires.
  8. Print final position and heading error to the recorded home pose.

Usage (inside the running sim, CycloneDDS env):
  python3 return_home_failsafe_scenario.py \\
      --init-x 0.0 --init-y 0.0 \\
      --dx 30 --dy 0 --yaw 1.5708 \\
      --drive-timeout 60 --timeout 120
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from autoware_nav2_offroad_msgs.msg import ReturnHomeState
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

# ReturnHomeState.result values (from autoware_nav2_offroad_msgs/msg/ReturnHomeState):
#   NONE        = 0
#   IN_PROGRESS = 1
#   REACHED     = 2
#   CANCELED    = 3
#   FAILED      = 4
RESULT_REACHED = 2
RESULT_CANCELED = 3
RESULT_FAILED = 4


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ReturnHomeScenario(Node):
    def __init__(self, args):
        super().__init__("return_home_failsafe_scenario")
        self.args = args
        self.ego = None
        self.rth_status = None

        self.sub_odom = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom, 10
        )
        self.sub_status = self.create_subscription(
            ReturnHomeState, "/return_home/status", self._on_status, 10
        )

        self.goal_pub = self.create_publisher(PoseStamped, "/planning/offroad_goal", 1)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 1
        )

        self.set_home_cli = self.create_client(Trigger, "/return_home/set_home")
        self.return_home_cli = self.create_client(Trigger, "/return_home/return_home")

    def _on_odom(self, msg):
        self.ego = msg

    def _on_status(self, msg):
        self.rth_status = msg

    # ------------------------------------------------------------------
    # Utility helpers (adapted from offroad_goal_scenario.py)
    # ------------------------------------------------------------------

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_ego(self, timeout):
        end = time.time() + timeout
        while self.ego is None and time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.ego is not None

    def set_initial_pose(self, x, y):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(msg)
            self.spin_for(0.2)

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(msg)

    def call_trigger(self, client, name, wait_sec=5.0):
        if not client.wait_for_service(timeout_sec=wait_sec):
            self.get_logger().error(f"service {name} not available after {wait_sec:.0f}s")
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            self.get_logger().error(f"service {name} call timed out")
            return False
        resp = future.result()
        if not resp.success:
            self.get_logger().warn(f"{name} responded: {resp.message}")
        else:
            self.get_logger().info(f"{name} OK: {resp.message}")
        return resp.success

    # ------------------------------------------------------------------
    # Main scenario
    # ------------------------------------------------------------------

    def run(self):
        a = self.args

        # 1. Optional initial pose
        if a.init_x is not None and a.init_y is not None:
            self.get_logger().info(f"Setting initial pose ({a.init_x}, {a.init_y})")
            self.set_initial_pose(a.init_x, a.init_y)

        # 2. Wait for ego pose
        self.get_logger().info("Waiting for ego pose on /localization/kinematic_state ...")
        if not self.wait_ego(30.0):
            self.get_logger().error("No ego pose received within 30 s; aborting")
            return 1

        # Record home pose (current ego position)
        home_x = self.ego.pose.pose.position.x
        home_y = self.ego.pose.pose.position.y
        home_yaw = yaw_from_quat(self.ego.pose.pose.orientation)
        self.get_logger().info(
            f"Home recorded: x={home_x:.3f} y={home_y:.3f} yaw={math.degrees(home_yaw):.1f} deg"
        )

        # 3. Call set_home
        self.get_logger().info("Calling /return_home/set_home ...")
        if not self.call_trigger(self.set_home_cli, "/return_home/set_home"):
            # Deliberate: scenario harness continues best-effort so it still reports the resulting error.
            self.get_logger().warn("set_home returned failure (proceeding anyway)")

        # 4. Drive out: publish a goal offset from home
        drive_x = home_x + a.dx
        drive_y = home_y + a.dy
        drive_yaw = a.yaw
        self.get_logger().info(
            f"Publishing drive-out goal: ({drive_x:.2f}, {drive_y:.2f}) yaw={drive_yaw:.3f}"
        )
        self.send_goal(drive_x, drive_y, drive_yaw)

        # 5. Wait for vehicle to move out (or drive-timeout to elapse)
        self.get_logger().info(f"Waiting {a.drive_timeout:.0f}s for vehicle to drive out ...")
        self.spin_for(a.drive_timeout)

        ego_after = self.ego
        dist_moved = math.hypot(
            ego_after.pose.pose.position.x - home_x,
            ego_after.pose.pose.position.y - home_y,
        )
        self.get_logger().info(f"Vehicle moved {dist_moved:.2f} m from home")

        # 6. Trigger return_home
        self.get_logger().info("Calling /return_home/return_home ...")
        if not self.call_trigger(self.return_home_cli, "/return_home/return_home"):
            self.get_logger().warn("return_home returned failure; will still wait for status")

        # 7. Poll /return_home/status until REACHED or timeout
        self.get_logger().info(
            f"Waiting up to {a.timeout:.0f}s for ReturnHomeState.result == REACHED (2) ..."
        )
        deadline = time.time() + a.timeout
        final_result = None
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.rth_status is not None:
                r = self.rth_status.result
                if r == RESULT_REACHED:
                    final_result = r
                    break
                if r == RESULT_CANCELED:
                    self.get_logger().error("ReturnHomeState.result == CANCELED")
                    final_result = r
                    break
                if r == RESULT_FAILED:
                    self.get_logger().error("ReturnHomeState.result == FAILED")
                    final_result = r
                    break

        # 8. Report
        fx = self.ego.pose.pose.position.x
        fy = self.ego.pose.pose.position.y
        fyaw = yaw_from_quat(self.ego.pose.pose.orientation)
        pos_err = math.hypot(fx - home_x, fy - home_y)
        yaw_err = abs(norm_angle(fyaw - home_yaw))

        result_str = {
            None: "TIMEOUT (no REACHED/CANCELED/FAILED received)",
            RESULT_REACHED: "REACHED",
            RESULT_CANCELED: "CANCELED",
            RESULT_FAILED: "FAILED",
        }.get(final_result, f"UNKNOWN ({final_result})")

        print()
        print("==== return-to-home failsafe scenario result ====")
        print(f"home:   x={home_x:.3f} y={home_y:.3f} yaw={math.degrees(home_yaw):.1f} deg")
        print(f"final:  x={fx:.3f} y={fy:.3f} yaw={math.degrees(fyaw):.1f} deg")
        print(f"RESULT:          {result_str}")
        print(f"POSITION ERROR:  {pos_err:.3f} m")
        print(f"HEADING ERROR:   {math.degrees(yaw_err):.2f} deg")

        return 0 if final_result == RESULT_REACHED else 1


def main():
    p = argparse.ArgumentParser(
        description="Return-To-Home failsafe scenario: set_home, drive out, "
        "trigger return_home, measure final error."
    )
    p.add_argument(
        "--init-x",
        type=float,
        default=None,
        help="Initial pose X to inject via /initialpose (optional)",
    )
    p.add_argument(
        "--init-y",
        type=float,
        default=None,
        help="Initial pose Y to inject via /initialpose (optional)",
    )
    p.add_argument(
        "--dx",
        type=float,
        default=30.0,
        help="Drive-out goal offset in X from home (metres, default 30)",
    )
    p.add_argument(
        "--dy",
        type=float,
        default=0.0,
        help="Drive-out goal offset in Y from home (metres, default 0)",
    )
    p.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Drive-out goal heading (radians, default 0)",
    )
    p.add_argument(
        "--drive-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for vehicle to drive out before triggering RTH (default 60)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for ReturnHomeState.result==REACHED after triggering RTH (default 120)",
    )
    args = p.parse_args()

    rclpy.init()
    node = ReturnHomeScenario(args)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
