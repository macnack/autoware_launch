# Reverse driving (`allow_reverse`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in `allow_reverse:=true` flag on `local_layer:=mppi_recovery` that enables reverse driving end-to-end (REEDS_SHEPP planning, MPPI reverse sampling, stop-and-shift gear sequencing in the bridge, `backup` recovery), so goals behind the vehicle become reachable.

**Architecture:** A `reverse_active` launch `<let>` (`mppi_recovery` AND `allow_reverse`) selects overlay param files and BT xmls via cascading `<let>` overrides — values always come from yaml files (type-safe), never from string `<param value>` overrides of typed doubles (a known silent no-op). The only new C++ is a pure `GearArbiter` unit inside the cmd_vel bridge implementing full stop-and-shift gear sequencing, TDD'd with gtest.

**Tech Stack:** ROS 2 Humble, Nav2 (`nav2_smac_planner` REEDS_SHEPP, `nav2_mppi_controller`, `nav2_behaviors` BackUp), BT.CPP v3, C++17 + gtest, XML launch.

## Global Constraints

- Build and test ONLY inside the `autoware` container, never on the host.
- `git add` only the files listed in each task's commit; never `git add -A`.
- End commit messages with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Default behavior (`allow_reverse:=false`, and all of `bridge`/`mppi`/`mppi_recovery` as they exist) must be byte-for-byte behaviorally unchanged; existing gtests stay green.
- Spec values: MPPI `vx_min: -1.5`; stop threshold `0.1` m/s; deadband `0.05` m/s; `bridge enable_reverse` default `false`.
- All paths relative to package root `tier4_universe_launch/autoware_nav2_offroad/` in the `_mppi_drive` worktree (branch `feat/offroad-mppi-drive`).
- Container build command (workspace): `colcon build --symlink-install --packages-select autoware_nav2_offroad --base-paths /workspace/_mppi_drive/tier4_universe_launch --build-base /workspace/_mppi_drive/_build --install-base /workspace/_mppi_drive/_install`
- Container test command: same but `colcon test` + `colcon test-result --verbose --test-result-base /workspace/_mppi_drive/_build/autoware_nav2_offroad`
- Runtime env for any live command: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///workspace/.devcontainer/cyclonedds_config.xml`, source `/opt/ros/humble/setup.bash`, `/workspace/install/setup.bash`, `/workspace/_mppi_drive/_install/setup.bash`, and `ros2 daemon stop && ros2 daemon start` before graph queries.

---

### Task 1: GearArbiter pure unit (TDD)

**Files:**
- Create: `include/autoware_nav2_offroad/gear_arbiter.hpp`
- Create: `src/gear_arbiter.cpp`
- Test: `test/test_gear_arbiter.cpp`
- Modify: `CMakeLists.txt:195-196` (add source to the existing `autoware_nav2_offroad_cmd_vel_to_control` library) and the test section (register `test_gear_arbiter`)

**Interfaces:**
- Produces: `namespace autoware::nav2_offroad`: `enum class Gear { DRIVE, REVERSE }`; `struct GearArbiterOut { Gear gear; double velocity_mps; }`; `class GearArbiter { GearArbiter(double stop_threshold_mps, double deadband_mps); GearArbiterOut update(double cmd_v_mps, double vehicle_speed_mps); }`. Task 2's bridge node consumes exactly these.

- [ ] **Step 1: Write the failing test**

Create `test/test_gear_arbiter.cpp`:

```cpp
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

#include "autoware_nav2_offroad/gear_arbiter.hpp"

#include <gtest/gtest.h>

namespace an = autoware::nav2_offroad;

namespace
{
an::GearArbiter makeArbiter() { return an::GearArbiter(0.1, 0.05); }
}  // namespace

TEST(GearArbiter, forward_cmd_in_drive_passes_through)
{
  auto a = makeArbiter();
  const auto out = a.update(1.2, 1.0);
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 1.2);
}

TEST(GearArbiter, deadband_holds_gear_and_outputs_zero)
{
  auto a = makeArbiter();
  const auto out = a.update(0.02, 0.0);  // |cmd| < 0.05 deadband
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
}

TEST(GearArbiter, flip_while_rolling_holds_gear_and_commands_zero)
{
  auto a = makeArbiter();
  a.update(1.0, 1.0);                     // established DRIVE, rolling
  const auto out = a.update(-1.0, 0.8);   // reverse requested, still rolling
  EXPECT_EQ(out.gear, an::Gear::DRIVE);   // NO gear change while |speed| >= 0.1
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
}

TEST(GearArbiter, flip_shifts_only_after_stop_then_passes_reverse)
{
  auto a = makeArbiter();
  a.update(1.0, 1.0);
  a.update(-1.0, 0.5);                    // hold: still rolling
  a.update(-1.0, 0.2);                    // hold: still rolling
  const auto out = a.update(-1.0, 0.05);  // |speed| < 0.1 -> shift + pass
  EXPECT_EQ(out.gear, an::Gear::REVERSE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, -1.0);
}

TEST(GearArbiter, reverse_to_drive_is_symmetric)
{
  auto a = makeArbiter();
  a.update(-1.0, 0.0);                    // shift to REVERSE at standstill
  a.update(-1.0, -0.8);                   // rolling backwards
  auto out = a.update(0.8, -0.5);         // forward requested while rolling back
  EXPECT_EQ(out.gear, an::Gear::REVERSE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.0);
  out = a.update(0.8, -0.04);             // nearly stopped -> shift + pass
  EXPECT_EQ(out.gear, an::Gear::DRIVE);
  EXPECT_DOUBLE_EQ(out.velocity_mps, 0.8);
}

TEST(GearArbiter, no_gear_change_is_ever_emitted_while_rolling)
{
  auto a = makeArbiter();
  a.update(1.0, 1.5);
  for (double sp = 1.5; sp >= 0.1; sp -= 0.1) {
    const auto out = a.update(-1.0, sp);
    EXPECT_EQ(out.gear, an::Gear::DRIVE) << "shifted at speed " << sp;
  }
}
```

- [ ] **Step 2: Register the test + library source in CMakeLists, run to verify it fails**

In `CMakeLists.txt`, extend the library (line 195):

```cmake
add_library(autoware_nav2_offroad_cmd_vel_to_control
  src/cmd_vel_to_control.cpp
  src/gear_arbiter.cpp
)
```

In the test section (after the `test_cmd_vel_to_control` block, before `endif()`):

```cmake
  ament_add_gtest(test_gear_arbiter
    test/test_gear_arbiter.cpp
  )
  if(TARGET test_gear_arbiter)
    target_link_libraries(test_gear_arbiter
      autoware_nav2_offroad_cmd_vel_to_control
    )
    target_include_directories(test_gear_arbiter PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
```

Run (container): the Global Constraints build command.
Expected: FAIL — `gear_arbiter.hpp: No such file or directory` (test written first).

- [ ] **Step 3: Write the implementation**

Create `include/autoware_nav2_offroad/gear_arbiter.hpp`:

```cpp
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

#ifndef AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_
#define AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_

namespace autoware::nav2_offroad
{
enum class Gear { DRIVE, REVERSE };

struct GearArbiterOut
{
  Gear gear{Gear::DRIVE};
  double velocity_mps{0.0};
};

/// Stop-and-shift gear arbitration for a reverse-capable cmd_vel bridge.
/// On a commanded direction flip, outputs zero velocity and holds the current
/// gear until the vehicle is nearly stopped (|speed| < stop_threshold), then
/// shifts and passes the new-direction command through. Pure unit: no ROS.
class GearArbiter
{
public:
  GearArbiter(double stop_threshold_mps, double deadband_mps)
  : stop_threshold_mps_(stop_threshold_mps), deadband_mps_(deadband_mps)
  {
  }

  GearArbiterOut update(double cmd_v_mps, double vehicle_speed_mps);

private:
  Gear gear_{Gear::DRIVE};
  double stop_threshold_mps_;
  double deadband_mps_;
};
}  // namespace autoware::nav2_offroad
#endif  // AUTOWARE_NAV2_OFFROAD__GEAR_ARBITER_HPP_
```

Create `src/gear_arbiter.cpp`:

```cpp
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

#include "autoware_nav2_offroad/gear_arbiter.hpp"

#include <cmath>

namespace autoware::nav2_offroad
{
GearArbiterOut GearArbiter::update(double cmd_v_mps, double vehicle_speed_mps)
{
  // Deadband: no meaningful command -> hold gear, command stop.
  if (std::abs(cmd_v_mps) < deadband_mps_) {
    return {gear_, 0.0};
  }
  const Gear desired = cmd_v_mps > 0.0 ? Gear::DRIVE : Gear::REVERSE;
  if (desired == gear_) {
    return {gear_, cmd_v_mps};
  }
  // Direction flip requested: hold current gear + command stop until nearly
  // stationary, then shift and pass the new-direction command through.
  if (std::abs(vehicle_speed_mps) >= stop_threshold_mps_) {
    return {gear_, 0.0};
  }
  gear_ = desired;
  return {gear_, cmd_v_mps};
}
}  // namespace autoware::nav2_offroad
```

Note (spec §4.4 nuance): the spec says "the next cycle passes the new-direction command"; this passes it in the same cycle the shift happens — the vehicle is already below the stop threshold, so the safety property (no shift while rolling) is identical and the unit is simpler.

- [ ] **Step 4: Build + run tests to verify they pass**

Run (container): build command, then test command.
Expected: `test_gear_arbiter` PASS (6 tests), and ALL existing tests still pass (130+ prior).

- [ ] **Step 5: Commit**

```bash
git add include/autoware_nav2_offroad/gear_arbiter.hpp src/gear_arbiter.cpp test/test_gear_arbiter.cpp CMakeLists.txt
git commit -m "feat(nav2_offroad): GearArbiter stop-and-shift gear unit (TDD)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Bridge node integration (`enable_reverse`)

**Files:**
- Modify: `src/cmd_vel_to_control_bridge_node.cpp`
- Modify: `CMakeLists.txt` (only if `nav_msgs` is missing from `ament_target_dependencies(cmd_vel_to_control_bridge_node ...)` — check first; `package.xml` already depends on `nav_msgs`)

**Interfaces:**
- Consumes: `GearArbiter` from Task 1 (exact signatures above).
- Produces: bridge node parameter `enable_reverse` (bool, default `false`), subscription `~/input/kinematic_state` (`nav_msgs/msg/Odometry`), and gear output that can be `GearCommand::REVERSE`. Task 4's launch consumes the param name and the remap.

- [ ] **Step 1: Modify the bridge node**

In `src/cmd_vel_to_control_bridge_node.cpp`, apply these exact edits:

Add includes (after the existing `#include "autoware_nav2_offroad/cmd_vel_to_control.hpp"`):

```cpp
#include "autoware_nav2_offroad/gear_arbiter.hpp"
```

and with the message includes:

```cpp
#include <nav_msgs/msg/odometry.hpp>
```

In the constructor, after `enabled_ = declare_parameter<bool>("initial_enabled", false);` block, add:

```cpp
    enable_reverse_ = declare_parameter<bool>("enable_reverse", false);
    if (enable_reverse_) {
      const double stop_thr = declare_parameter<double>("gear_stop_threshold_mps", 0.1);
      const double deadband = declare_parameter<double>("gear_deadband_mps", 0.05);
      arbiter_ = std::make_unique<GearArbiter>(stop_thr, deadband);
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "~/input/kinematic_state", rclcpp::QoS(1),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          vehicle_speed_mps_ = msg->twist.twist.linear.x;
        });
      RCLCPP_INFO(get_logger(), "reverse ENABLED (stop-and-shift gear sequencing)");
    }
```

Replace the body of `onCmdVel` from `const auto c = twistToControl(...)` down to the gear publish with:

```cpp
    double cmd_v = msg->linear.x;
    uint8_t gear_cmd = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    if (enable_reverse_) {
      const auto arb = arbiter_->update(cmd_v, vehicle_speed_mps_);
      cmd_v = arb.velocity_mps;
      gear_cmd = arb.gear == Gear::REVERSE
        ? autoware_vehicle_msgs::msg::GearCommand::REVERSE
        : autoware_vehicle_msgs::msg::GearCommand::DRIVE;
    }
    const auto c = twistToControl(cmd_v, msg->angular.z, params_, last_steer_);
    last_steer_ = c.steering_tire_angle_rad;
    autoware_control_msgs::msg::Control ctrl;
    ctrl.stamp = now();
    ctrl.longitudinal.velocity = static_cast<float>(c.velocity_mps);
    ctrl.lateral.steering_tire_angle = static_cast<float>(c.steering_tire_angle_rad);
    pub_ctrl_->publish(ctrl);
    autoware_vehicle_msgs::msg::GearCommand gear;
    gear.stamp = ctrl.stamp;
    gear.command = gear_cmd;
    pub_gear_->publish(gear);
```

Add member variables (with the existing ones):

```cpp
  bool enable_reverse_{false};
  double vehicle_speed_mps_{0.0};
  std::unique_ptr<GearArbiter> arbiter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
```

(`<memory>` is already included.)

- [ ] **Step 2: Ensure `nav_msgs` is in the bridge target deps**

Check `CMakeLists.txt` `ament_target_dependencies(cmd_vel_to_control_bridge_node ...)` (near line 212). If `nav_msgs` is not listed, add it to that list.

- [ ] **Step 3: Build + all tests green**

Run: build + test commands.
Expected: builds clean; all gtests (incl. `test_gear_arbiter` and the 5 existing `test_cmd_vel_to_control`) PASS. With `enable_reverse=false` (default) the node's behavior is unchanged: gear is constant `DRIVE`, velocity passthrough untouched.

- [ ] **Step 4: Commit**

```bash
git add src/cmd_vel_to_control_bridge_node.cpp CMakeLists.txt
git commit -m "feat(nav2_offroad): bridge enable_reverse — GearArbiter wiring + odometry feedback

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Reverse config files (overlays + behavior config + recovery BTs)

**Files:**
- Create: `config/nav2_offroad_forward.param.yaml`, `config/nav2_offroad_reverse.param.yaml`
- Create: `config/nav2_mppi_forward.param.yaml`, `config/nav2_mppi_reverse.param.yaml`
- Create: `config/nav2_behavior_server_reverse.param.yaml`
- Create: `config/behavior_trees/navigate_to_pose_recovery_reverse_offroad.xml`, `config/behavior_trees/navigate_through_poses_recovery_reverse_offroad.xml`

**Interfaces:**
- Produces: five yaml files + two BT xmls whose exact paths Task 4's launch `<let>`s reference. The forward overlays restate the current forward values so BOTH branches layer a file (no conditional `<param from>` needed — the known-broken string `<param value>` override is avoided entirely).

- [ ] **Step 1: Planner overlays**

`config/nav2_offroad_forward.param.yaml`:

```yaml
# Planner motion-model overlay, forward-only (default). Layered over
# nav2_offroad.param.yaml on planner_server; restates the forward value so the
# reverse/forward selection is a <let>-chosen FILE (type-safe), never a string
# <param value> override (silently fails on typed params).
planner_server:
  ros__parameters:
    GridBased:
      motion_model_for_search: "DUBIN"
```

`config/nav2_offroad_reverse.param.yaml`:

```yaml
# Planner motion-model overlay for allow_reverse:=true (mppi_recovery only).
# REEDS_SHEPP plans forward AND reverse arcs, so poses behind the vehicle
# become reachable. reverse_penalty stays as set in nav2_offroad.param.yaml.
planner_server:
  ros__parameters:
    GridBased:
      motion_model_for_search: "REEDS_SHEPP"
```

- [ ] **Step 2: Controller overlays**

`config/nav2_mppi_forward.param.yaml`:

```yaml
# MPPI velocity-range overlay, forward-only (default): restates vx_min 0.0.
controller_server:
  ros__parameters:
    FollowPath:
      vx_min: 0.0
```

`config/nav2_mppi_reverse.param.yaml`:

```yaml
# MPPI velocity-range overlay for allow_reverse:=true: allow reverse sampling.
# Reverse capped slower than forward (vx_max 2.5) — cautious by default.
controller_server:
  ros__parameters:
    FollowPath:
      vx_min: -1.5
```

- [ ] **Step 3: behavior_server reverse config**

`config/nav2_behavior_server_reverse.param.yaml` — copy of `config/nav2_behavior_server.param.yaml` with `backup` added; full content:

```yaml
# behavior_server (nav2_behaviors) for local_layer:=mppi_recovery with
# allow_reverse:=true. Adds the backup recovery (reverse is available); still
# NO spin (an Ackermann vehicle cannot rotate in place).
behavior_server:
  ros__parameters:
    use_sim_time: false
    local_costmap_topic: /local_costmap/costmap_raw
    local_footprint_topic: /local_costmap/published_footprint
    global_costmap_topic: /global_costmap/costmap_raw
    global_footprint_topic: /global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["wait", "drive_on_heading", "backup"]
    wait:
      plugin: "nav2_behaviors/Wait"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    backup:
      plugin: "nav2_behaviors/BackUp"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.2
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.0
    min_rotational_vel: 0.0
    rotational_acc_lim: 0.0
```

(NOTE the plugin type format: `nav2_behaviors/BackUp` with a slash — the `::` form fails to load; this bit us once already.)

- [ ] **Step 4: Reverse recovery BTs**

`config/behavior_trees/navigate_to_pose_recovery_reverse_offroad.xml`:

```xml
<!--
  Reverse-capable NavigateToPose recovery BT (allow_reverse:=true, EXPERIMENTAL).
  Same retry loop as the forward recovery BT, with a BackUp recovery between the
  costmap clears and the wait: when planning/following fails (typically because a
  forward-only approach overshot), back up ~1 m and retry. Still no Spin.
  BT.CPP v3 (ROS 2 Humble); planner_id/controller_id match nav2_offroad.param.yaml
  and nav2_mppi_controller.param.yaml.
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <Sequence name="RecoveryActions">
        <ClearEntireCostmap name="ClearGlobalCostmap-Context"
                            service_name="global_costmap/clear_entirely_global_costmap"/>
        <ClearEntireCostmap name="ClearLocalCostmap-Context"
                            service_name="local_costmap/clear_entirely_local_costmap"/>
        <BackUp backup_dist="1.0" backup_speed="0.3"/>
        <Wait wait_duration="2.0"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

`config/behavior_trees/navigate_through_poses_recovery_reverse_offroad.xml` — identical structure with the through-poses drive tree:

```xml
<!--
  Reverse-capable NavigateThroughPoses recovery BT (allow_reverse:=true).
  See navigate_to_pose_recovery_reverse_offroad.xml; ComputePathThroughPoses(goals).
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <Sequence name="RecoveryActions">
        <ClearEntireCostmap name="ClearGlobalCostmap-Context"
                            service_name="global_costmap/clear_entirely_global_costmap"/>
        <ClearEntireCostmap name="ClearLocalCostmap-Context"
                            service_name="local_costmap/clear_entirely_local_costmap"/>
        <BackUp backup_dist="1.0" backup_speed="0.3"/>
        <Wait wait_duration="2.0"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

- [ ] **Step 5: Verify files (yaml parse, xml well-formed, no Spin)**

Run (package root):
```bash
python3 - <<'PY'
import yaml, xml.dom.minidom as x
for f in ["config/nav2_offroad_forward.param.yaml","config/nav2_offroad_reverse.param.yaml",
          "config/nav2_mppi_forward.param.yaml","config/nav2_mppi_reverse.param.yaml",
          "config/nav2_behavior_server_reverse.param.yaml"]:
    yaml.safe_load(open(f)); print("yaml ok:", f)
for f in ["config/behavior_trees/navigate_to_pose_recovery_reverse_offroad.xml",
          "config/behavior_trees/navigate_through_poses_recovery_reverse_offroad.xml"]:
    x.parse(f); print("xml ok:", f)
PY
grep -hoE "<Spin\b" config/behavior_trees/navigate_*_reverse_offroad.xml && echo "SPIN FOUND (bad)" || echo "no Spin (ok)"
```
Expected: all `ok`, `no Spin (ok)`.

- [ ] **Step 6: Commit**

```bash
git add config/nav2_offroad_forward.param.yaml config/nav2_offroad_reverse.param.yaml config/nav2_mppi_forward.param.yaml config/nav2_mppi_reverse.param.yaml config/nav2_behavior_server_reverse.param.yaml config/behavior_trees/navigate_to_pose_recovery_reverse_offroad.xml config/behavior_trees/navigate_through_poses_recovery_reverse_offroad.xml
git commit -m "feat(nav2_offroad): reverse overlays (REEDS_SHEPP, vx_min -1.5), backup behavior config, reverse recovery BTs

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: Launch integration — `allow_reverse`

**Files:**
- Modify: `launch/nav2_offroad.launch.xml`

**Interfaces:**
- Consumes: the seven files from Task 3 (exact paths) and the bridge `enable_reverse` param from Task 2.
- Produces: launch arg `allow_reverse` (default `false`); `reverse_active` semantics = `local_layer==mppi_recovery AND allow_reverse` (documented: ignored in other modes).

Pattern: cascading `<let>` overrides — a later `<let>` of the same name replaces the earlier one, so the third (reverse) assignment simply overrides the mppi_recovery one when reverse is active. All values come from files.

- [ ] **Step 1: Add the arg + `reverse_active` let**

After the `local_layer` arg (line ~33), add:

```xml
  <arg name="allow_reverse" default="false"
       description="mppi_recovery only (EXPERIMENTAL): enable reverse driving — REEDS_SHEPP planning, MPPI vx_min<0, stop-and-shift gear sequencing, backup recovery. Ignored in other modes."/>
```

Right before the `lifecycle_nodes` lets, add:

```xml
  <!-- reverse is only meaningful in mppi_recovery; all reverse selections key off this -->
  <let name="reverse_active"
       value="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot; and &quot;$(var allow_reverse)&quot;==&quot;true&quot;')"/>
```

- [ ] **Step 2: Extend the BT xml cascade with reverse overrides**

Immediately after the existing four `bt_to_pose_xml` / `bt_through_poses_xml` lets, add:

```xml
  <let name="bt_to_pose_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_to_pose_recovery_reverse_offroad.xml"
       if="$(var reverse_active)"/>
  <let name="bt_through_poses_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_through_poses_recovery_reverse_offroad.xml"
       if="$(var reverse_active)"/>
```

- [ ] **Step 3: Add file-selection lets for planner/controller/behavior configs**

After the BT lets, add:

```xml
  <!-- reverse/forward selection is always a FILE chosen by <let> — a string
       <param value> cannot override a typed double (silent no-op; bit us once). -->
  <let name="planner_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_offroad_forward.param.yaml"/>
  <let name="planner_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_offroad_reverse.param.yaml"
       if="$(var reverse_active)"/>
  <let name="mppi_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_mppi_forward.param.yaml"/>
  <let name="mppi_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_mppi_reverse.param.yaml"
       if="$(var reverse_active)"/>
  <let name="behavior_server_param_file"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_behavior_server.param.yaml"/>
  <let name="behavior_server_param_file"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_behavior_server_reverse.param.yaml"
       if="$(var reverse_active)"/>
```

- [ ] **Step 4: Layer the overlays on the nodes**

On BOTH `planner_server` nodes (hybrid line ~91 and lattice line ~100), add after their existing `<param from=...>` lines:

```xml
    <param from="$(var planner_motion_overlay)"/>
```

On the `controller_server` node (~line 126), after `<param from="$(var mppi_param_file)"/>`:

```xml
    <param from="$(var mppi_motion_overlay)"/>
```

On the `behavior_server` node (~line 146), replace the hardcoded param file:

```xml
    <param from="$(var behavior_server_param_file)"/>
```

- [ ] **Step 5: Wire the bridge**

On the `cmd_vel_to_control_bridge` node (~line 162), add with its params/remaps:

```xml
    <param name="enable_reverse" value="$(var reverse_active)"/>
    <remap from="~/input/kinematic_state" to="/localization/kinematic_state"/>
```

(`enable_reverse` is a bool — yaml parses `True`/`False` reliably; the type trap is doubles. Verified live in Task 6 anyway.)

- [ ] **Step 6: Verify + build + parse all combinations**

Run (package root): `xmllint --noout launch/nav2_offroad.launch.xml`
Then (container): build command, then:

```bash
for args in "local_layer:=bridge" "local_layer:=mppi" "local_layer:=mppi_recovery" \
            "local_layer:=mppi_recovery allow_reverse:=true" "local_layer:=mppi allow_reverse:=true"; do
  echo -n "  [$args] "; timeout 30 ros2 launch autoware_nav2_offroad nav2_offroad.launch.xml $args --show-args >/dev/null 2>&1 && echo OK || echo ERR
done
```
Expected: all five `OK` (the last confirms `allow_reverse` is harmlessly ignored outside `mppi_recovery` — `reverse_active` stays false).

- [ ] **Step 7: Commit**

```bash
git add launch/nav2_offroad.launch.xml
git commit -m "feat(nav2_offroad): allow_reverse flag on mppi_recovery — file-selected reverse overlays + bridge wiring

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: Documentation

**Files:**
- Modify: `README.md` (local_layer/args table + the `mppi_recovery` experimental section)
- Modify: `BACKLOG.md` (the `mppi_recovery` follow-ups bullet)

- [ ] **Step 1: README args table**

Add a row after the `local_layer` row:

```markdown
| `allow_reverse` | `true` \| `false` | `false` | `mppi_recovery` only (**EXPERIMENTAL**): reverse driving — REEDS_SHEPP planning, MPPI `vx_min: -1.5`, stop-and-shift gear sequencing in the bridge (holds a stop until \|speed\| < 0.1 m/s before any DRIVE↔REVERSE change), `backup` recovery. Ignored in other modes. |
```

- [ ] **Step 2: README experimental section**

Append to the `### Experimental: local_layer:=mppi_recovery` section:

```markdown
#### Reverse (`allow_reverse:=true`)

Forward-only DUBIN cannot turn around, so goals behind the vehicle are
unreachable. `allow_reverse:=true` enables reverse end-to-end: the planner
switches to REEDS_SHEPP (forward+reverse arcs), MPPI samples reverse
(`vx_min: -1.5`), the cmd_vel bridge runs a stop-and-shift gear state machine
(zero-velocity hold of the current gear until |speed| < 0.1 m/s, then shift —
hardware never sees a direction slam), and the recovery BT gains a `BackUp`
step. Default `false` keeps `mppi_recovery` forward-only and unchanged.
```

- [ ] **Step 3: BACKLOG update**

In the `mppi_recovery` follow-ups bullet, replace item (a):

from `(a) reverse variant (REEDS_SHEPP planner + MPPI reverse + backup recovery) to fix exact-pose alignment that forward-only DUBIN cannot;`
to `(a) ~~reverse variant~~ DONE via allow_reverse:=true (REEDS_SHEPP + vx_min -1.5 + stop-and-shift gear sequencing + backup recovery); remaining: reverse speed/penalty tuning;`

- [ ] **Step 4: Commit**

```bash
git add README.md BACKLOG.md
git commit -m "docs(nav2_offroad): document allow_reverse (EXPERIMENTAL reverse driving)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: Runtime acceptance — goal behind the vehicle

No unit test possible (needs the full planning simulator). Documented, repeatable procedure in the `autoware` container; requires the user to relaunch (`task_recovery.sh` with `allow_reverse:=true` added) since the sim occupies the DDS graph.

- [ ] **Step 1: Relaunch with reverse**

Add `allow_reverse:=true` to the `task_recovery.sh` launch line; user relaunches. Wait for bringup.

- [ ] **Step 2: Verify the reverse config took (params are live-verified, not assumed)**

```bash
ros2 param get /planner_server GridBased.motion_model_for_search   # expect REEDS_SHEPP
ros2 param get /controller_server FollowPath.vx_min                # expect -1.5
ros2 param get /cmd_vel_to_control_bridge enable_reverse           # expect true
ros2 lifecycle get /behavior_server                                # expect active
ros2 action list | grep backup                                     # expect /backup
```

- [ ] **Step 3: Drive a goal BEHIND the vehicle**

Re-establish the LOCAL drive path (relay), reset pose, publish an off-road goal ~10 m directly behind the current pose (goal yaw = vehicle yaw, position behind). Track ego + `/offroad_goal_relay/result` + the bridge gear output (`ros2 topic echo /planning/gear_cmd` — expect `command: 20` (REVERSE) episodes with zero-velocity holds around each gear change).
Expected: the vehicle maneuvers using at least one reverse segment and `result` reaches SUCCEEDED — the exact case forward-only could not do. Record min-distance + gear transitions.

- [ ] **Step 4: Regression — `allow_reverse:=false`**

Relaunch without the flag: `ros2 param get /planner_server GridBased.motion_model_for_search` → `DUBIN`; `vx_min` → `0.0`; `enable_reverse` → `false`; behavior_server plugins have no `backup`. Forward goal still reaches (Task-5-of-recovery baseline).

- [ ] **Step 5: Record the result**

Note pass/fail + any tuning (reverse_penalty, vx_min, BackUp dist) in the PR description / acceptance note. If the behind-goal still fails, the failure now lands in tuning space (penalties/critics), not capability space.

---

## Self-Review

**1. Spec coverage:** §4.1 launch (arg, reverse_active, file-selection lets, bridge wiring) → Task 4. §4.2 planner overlay → Task 3 Step 1 + Task 4 Step 4. §4.3 controller overlay → Task 3 Step 2 + Task 4 Step 4. §4.4 GearArbiter + bridge → Tasks 1–2 (stop threshold 0.1, deadband 0.05, kinematic_state sub, enable_reverse default false, watchdog untouched). §4.5 backup behavior config + reverse BTs → Task 3 Steps 3–4 + Task 4 Steps 2–3. §5 safety (no slam, reverse slower, opt-in) → Tasks 1–3 content. §6 testing (GearArbiter gtest, launch-parse incl. ignored-flag case, regression, behind-goal acceptance) → Task 1, Task 4 Step 6, Task 6. ✔

**2. Placeholder scan:** none — every file's full content or exact edit is inline. ✔

**3. Type consistency:** `GearArbiter(double, double)` / `update(double, double)` / `GearArbiterOut{gear, velocity_mps}` / `Gear::DRIVE|REVERSE` identical in Task 1 test, Task 1 impl, and Task 2 usage. File paths in Task 3 match Task 4's lets verbatim. `enable_reverse` name matches Task 2 declare and Task 4 wiring. ✔

**Noted deviation from spec:** §4.4 said the post-shift command passes "the next cycle"; the implementation passes it in the same cycle the shift occurs (vehicle already below the stop threshold — same safety property, simpler unit). Flagged in Task 1 Step 3.
