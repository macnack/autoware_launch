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

"""Reproducible off-road goal scenario + final-goal error measurement.

Initializes localization, brings up Nav2, sends an off-road goal at
(ego + dx, ego + dy) with the requested heading, waits until the vehicle stops,
and reports the position and heading error to the goal.

Usage (inside the running sim, CycloneDDS env):
  python3 offroad_goal_scenario.py --dx 20 --dy 0 --yaw 1.5708
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Scenario(Node):
    def __init__(self, args):
        super().__init__("offroad_goal_scenario")
        self.args = args
        self.ego = None
        self.sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_odom, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/planning/offroad_goal", 1)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.lifecycle = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes")

    def _on_odom(self, msg):
        self.ego = msg

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

    def manage_nav2(self, command):
        if not self.lifecycle.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("lifecycle service unavailable")
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        self.lifecycle.call_async(req)
        self.spin_for(3.0)

    def send_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.goal_pub.publish(msg)

    def run(self):
        a = self.args
        if a.init_x is not None and a.init_y is not None:
            self.get_logger().info("setting initial pose")
            self.set_initial_pose(a.init_x, a.init_y)
        if not self.wait_ego(30.0):
            self.get_logger().error("no ego pose; aborting")
            return 1
        if a.startup_nav2:
            self.get_logger().info("STARTUP nav2 (command 0)")
            self.manage_nav2(ManageLifecycleNodes.Request.STARTUP)
            if not self.wait_ego(5.0):
                return 1

        ex = self.ego.pose.pose.position.x
        ey = self.ego.pose.pose.position.y
        gx, gy, gyaw = ex + a.dx, ey + a.dy, a.yaw
        self.get_logger().info(f"ego=({ex:.2f},{ey:.2f}) goal=({gx:.2f},{gy:.2f}) yaw={gyaw:.3f}")
        self.send_goal(gx, gy, gyaw)

        # wait until arrival (stopped near goal) or timeout
        deadline = time.time() + a.timeout
        stopped_since = None
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            v = self.ego.twist.twist.linear.x
            d = math.hypot(self.ego.pose.pose.position.x - gx,
                           self.ego.pose.pose.position.y - gy)
            if abs(v) < 0.05 and d < 3.0:
                stopped_since = stopped_since or time.time()
                if time.time() - stopped_since > 2.0:
                    break
            else:
                stopped_since = None

        fx = self.ego.pose.pose.position.x
        fy = self.ego.pose.pose.position.y
        fyaw = yaw_from_quat(self.ego.pose.pose.orientation)
        pos_err = math.hypot(fx - gx, fy - gy)
        yaw_err = abs(norm_angle(fyaw - gyaw))
        print("==== off-road goal scenario result ====")
        print(f"goal:   x={gx:.3f} y={gy:.3f} yaw={gyaw:.3f}")
        print(f"final:  x={fx:.3f} y={fy:.3f} yaw={fyaw:.3f}")
        print(f"POSITION ERROR: {pos_err:.3f} m")
        print(f"HEADING ERROR:  {math.degrees(yaw_err):.2f} deg")
        return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dx", type=float, default=20.0)
    p.add_argument("--dy", type=float, default=0.0)
    p.add_argument("--yaw", type=float, default=1.5708)
    p.add_argument("--init-x", type=float, default=None)
    p.add_argument("--init-y", type=float, default=None)
    p.add_argument("--startup-nav2", action="store_true")
    p.add_argument("--timeout", type=float, default=60.0)
    args = p.parse_args()

    rclpy.init()
    node = Scenario(args)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
