# MPPI Drivable Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `local_layer:=mppi` actually drive the vehicle: off-road goal → bt_navigator/MPPI → `/cmd_vel` → Control → `vehicle_cmd_gate` AUTO input.

**Architecture:** Two seams get closed. (1) A new `offroad_goal_relay_node` converts the `/planning/offroad_goal` topic into the `NavigateToPose` action bt_navigator expects (cancel via `/planning/offroad_cancel`). (2) The reviewed `cmd_vel_to_control_bridge` (bicycle model + stale-cmd_vel watchdog) is ported from `feat/teach-repeat-rth` and its `Control` output is routed to the gate's auto input via a new `auto_control_cmd_topic` launch arg that mirrors the existing `auto_gear_cmd_topic` precedent. v1 is forward-only (`vx_min: 0.0`).

**Tech Stack:** C++17, ROS 2 Humble (`rclcpp`, `rclcpp_action`), `nav2_msgs/action/NavigateToPose`, `autoware_control_msgs/Control`, gtest.

## Global Constraints

- **Branch/worktree:** implement on `feat/offroad-mppi-drive`, checked out at the worktree `/home/maciej/autoware/_mppi_drive` (container path `/workspace/_mppi_drive`). Do NOT touch the main checkout at `src/launcher/autoware_launch` (parallel sessions use it). `git add` only each task's files.
- **Build/test ONLY inside the `autoware` container**, isolated dirs (proven pattern):
  ```bash
  docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && colcon build --symlink-install --base-paths tier4_universe_launch --packages-up-to autoware_nav2_offroad --build-base _build --install-base _install'
  docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && colcon test --base-paths tier4_universe_launch --packages-select autoware_nav2_offroad --build-base _build --install-base _install --test-result-base _build && colcon test-result --test-result-base _build --verbose'
  ```
  Baseline before this feature: **114 tests, 0 failures** — must not regress.
- **Copyright header** (verbatim, every new `.cpp`/`.hpp`): the Apache-2.0 block starting `// Copyright 2026 Maciej Krupka maciej.krupka@put.poznan.pl` (copy from `src/mode_manager_core.cpp`). Namespace `autoware::nav2_offroad`.
- **Port, don't rewrite:** the cmd_vel bridge files are taken verbatim from `feat/teach-repeat-rth` via `git show` (the watchdog is included in their current state); only the extensions named in Task 1 are added.
- **v1 scope (spec §4):** forward-only (`vx_min: 0.0`); no BT customization; no lattice changes; no reverse gear sequencing; smoke tests headless (full sim drive = final acceptance, may be manual).
- Always `pkill` stray ROS nodes after smoke tests (project rule).

---

### Task 1: Port `cmd_vel_to_control_bridge` (+ `initial_enabled` extension)

**Files:**
- Create (ported): `tier4_universe_launch/autoware_nav2_offroad/include/autoware_nav2_offroad/cmd_vel_to_control.hpp`, `src/cmd_vel_to_control.cpp`, `src/cmd_vel_to_control_bridge_node.cpp`, `test/test_cmd_vel_to_control.cpp`, `config/cmd_vel_to_control.param.yaml`
- Modify: `tier4_universe_launch/autoware_nav2_offroad/CMakeLists.txt`, `package.xml`

**Interfaces:**
- Produces (Task 3 relies on): executable `cmd_vel_to_control_bridge_node`; sub `~/input/cmd_vel` (Twist); pub `~/output/control_cmd` (`autoware_control_msgs/Control`), `~/output/gear_cmd` (GearCommand DRIVE); srv `~/enable` (SetBool); params `wheelbase_m`, `max_steer_rad`, `min_speed_for_steer_mps`, `cmd_vel_timeout_s`, and NEW `initial_enabled` (bool, default false).

- [ ] **Step 1: Port the five files verbatim from the teach&repeat branch**

From the worktree root (`/home/maciej/autoware/_mppi_drive`) — the shared repo's refs are visible from the worktree:
```bash
P=tier4_universe_launch/autoware_nav2_offroad
for f in include/autoware_nav2_offroad/cmd_vel_to_control.hpp src/cmd_vel_to_control.cpp \
         src/cmd_vel_to_control_bridge_node.cpp test/test_cmd_vel_to_control.cpp \
         config/cmd_vel_to_control.param.yaml; do
  git show feat/teach-repeat-rth:$P/$f > $P/$f
done
```

- [ ] **Step 2: Add the `initial_enabled` extension**

In `src/cmd_vel_to_control_bridge_node.cpp`, in the constructor where `enabled_{false}` / params are declared, add:
```cpp
enabled_ = declare_parameter<bool>("initial_enabled", false);
if (enabled_) {
  RCLCPP_INFO(get_logger(), "bridge starts ENABLED (initial_enabled=true)");
}
```
(Place after the existing parameter declarations; keep the `~/enable` service unchanged.) Add `initial_enabled: false` to `config/cmd_vel_to_control.param.yaml` with a comment `# mppi mode launches with true`.

- [ ] **Step 3: Wire CMake + package.xml**

`CMakeLists.txt` — add, following the existing library/executable/gtest blocks:
```cmake
add_library(autoware_nav2_offroad_cmd_vel_to_control
  src/cmd_vel_to_control.cpp
)
target_include_directories(autoware_nav2_offroad_cmd_vel_to_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cmd_vel_to_control_bridge_node
  src/cmd_vel_to_control_bridge_node.cpp
)
target_link_libraries(cmd_vel_to_control_bridge_node
  autoware_nav2_offroad_cmd_vel_to_control
)
target_include_directories(cmd_vel_to_control_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(cmd_vel_to_control_bridge_node
  autoware_control_msgs
  autoware_vehicle_msgs
  geometry_msgs
  rclcpp
  std_srvs
)
```
Add both targets to `install(TARGETS ...)`. Add `find_package(autoware_control_msgs REQUIRED)`. In the `if(BUILD_TESTING)` block:
```cmake
  ament_add_gtest(test_cmd_vel_to_control
    test/test_cmd_vel_to_control.cpp
  )
  if(TARGET test_cmd_vel_to_control)
    target_link_libraries(test_cmd_vel_to_control
      autoware_nav2_offroad_cmd_vel_to_control
    )
    target_include_directories(test_cmd_vel_to_control PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
  endif()
```
`package.xml`: add `<depend>autoware_control_msgs</depend>` (verify `autoware_vehicle_msgs`, `std_srvs`, `geometry_msgs` present — they are on this branch).

- [ ] **Step 4: Build + run the ported gtests**

Run the Global Constraints build then test command with `--ctest-args -R test_cmd_vel_to_control` added to the test invocation.
Expected: build clean; **5/5** `TwistToControl` tests pass (StraightLineZeroSteer, BicycleModelSteer, ClampsToMaxSteer, ZeroSpeedHoldsLastSteer, ReverseVelocityProducesSignConsistentSteer).

- [ ] **Step 5: Smoke the `initial_enabled` extension**

```bash
docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && source _install/setup.bash && \
  ros2 run autoware_nav2_offroad cmd_vel_to_control_bridge_node --ros-args -p initial_enabled:=true & sleep 3; \
  ros2 topic pub --once /cmd_vel_to_control_bridge/input/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"; \
  timeout 3 ros2 topic echo --once /cmd_vel_to_control_bridge/output/control_cmd; \
  pkill -f cmd_vel_to_control_bridge_node'
```
Expected: a `Control` message echoes (longitudinal.velocity 1.0) WITHOUT any `~/enable` call — proving `initial_enabled` works.

- [ ] **Step 6: Commit**

```bash
git add tier4_universe_launch/autoware_nav2_offroad/include tier4_universe_launch/autoware_nav2_offroad/src tier4_universe_launch/autoware_nav2_offroad/test tier4_universe_launch/autoware_nav2_offroad/config/cmd_vel_to_control.param.yaml tier4_universe_launch/autoware_nav2_offroad/CMakeLists.txt tier4_universe_launch/autoware_nav2_offroad/package.xml
git commit -m "feat(nav2_offroad): port cmd_vel_to_control bridge from teach&repeat + initial_enabled param"
```

---

### Task 2: `offroad_goal_relay_node` (goal topic → NavigateToPose action)

**Files:**
- Create: `tier4_universe_launch/autoware_nav2_offroad/src/offroad_goal_relay_node.cpp`
- Modify: `tier4_universe_launch/autoware_nav2_offroad/CMakeLists.txt`

**Interfaces:**
- Consumes: bt_navigator's `navigate_to_pose` action server (`nav2_msgs/action/NavigateToPose`).
- Produces (Task 3 relies on): executable `offroad_goal_relay_node`; sub `~/input/goal` (PoseStamped) and `~/input/cancel` (`std_msgs/Bool`); pub `~/result` (`std_msgs/UInt8`, transient_local: 0 NONE, 1 ACTIVE, 2 SUCCEEDED, 3 ABORTED, 4 CANCELED; publishes 0 at startup).
- Semantics: a NEW goal while one is in flight **preempts** it (bt_navigator replaces the active NavigateToPose goal) — matching the off-road-goal "latest goal wins" behavior of the bridge mode. No re-entrancy rejection here, unlike the NavThroughPoses bridge.

- [ ] **Step 1: Implement the node (full file)**

`src/offroad_goal_relay_node.cpp`:
```cpp
// Copyright 2026 Maciej Krupka maciej.krupka@put.poznan.pl
// ... (full Apache-2.0 header copied from src/mode_manager_core.cpp)
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
```

- [ ] **Step 2: CMake**

```cmake
add_executable(offroad_goal_relay_node
  src/offroad_goal_relay_node.cpp
)
ament_target_dependencies(offroad_goal_relay_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  std_msgs
)
```
Add to `install(TARGETS ...)`. (`nav2_msgs`, `rclcpp_action`, `std_msgs` are already package deps on this branch.)

- [ ] **Step 3: Build**

Global Constraints build command. Expected: clean.

- [ ] **Step 4: Smoke — no action server → ABORTED; startup NONE retained**

```bash
docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && source _install/setup.bash && \
  ros2 run autoware_nav2_offroad offroad_goal_relay_node & sleep 3; \
  timeout 2 ros2 topic echo --once /offroad_goal_relay/result; \
  ros2 topic pub --once /offroad_goal_relay/input/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 5.0}, orientation: {w: 1.0}}}"; \
  sleep 7; timeout 2 ros2 topic echo --once /offroad_goal_relay/result; \
  pkill -f offroad_goal_relay_node'
```
Expected: first echo `data: 0` (startup NONE); second echo `data: 3` (ABORTED after the 5 s server wait).

- [ ] **Step 5: Commit**

```bash
git add tier4_universe_launch/autoware_nav2_offroad/src/offroad_goal_relay_node.cpp tier4_universe_launch/autoware_nav2_offroad/CMakeLists.txt
git commit -m "feat(nav2_offroad): add offroad_goal_relay (goal topic -> NavigateToPose, latest-goal-wins)"
```

---

### Task 3: mppi-mode launch wiring + forward-only v1

**Files:**
- Modify: `tier4_universe_launch/autoware_nav2_offroad/launch/nav2_offroad.launch.xml`
- Modify: `tier4_universe_launch/autoware_nav2_offroad/config/nav2_mppi_controller.param.yaml`

**Interfaces:**
- Consumes: Task 1 + Task 2 executables. Produces: in mppi mode, `Control` on `/nav2_offroad/mppi/control_cmd` and gear on `/planning/gear_cmd` (the existing offroad `auto_gear_cmd_topic` value); Task 4 routes the gate to read it.

- [ ] **Step 1: Forward-only v1**

In `config/nav2_mppi_controller.param.yaml` change line 41:
```yaml
      vx_min: 0.0          # v1 forward-only: reverse needs gear sequencing in the bridge (follow-up)
```

- [ ] **Step 2: Launch the two nodes in the mppi branch**

In `launch/nav2_offroad.launch.xml`, next to the existing mppi-gated nodes, add (matching their `if=` gating style):
```xml
  <arg name="vehicle_wheelbase_m" default="2.7"/>
  <arg name="cmd_vel_bridge_param_file" default="$(find-pkg-share autoware_nav2_offroad)/config/cmd_vel_to_control.param.yaml"/>

  <!-- mppi mode: goal topic -> NavigateToPose action (latest goal wins; cancel reuses offroad_cancel) -->
  <node pkg="autoware_nav2_offroad" exec="offroad_goal_relay_node" name="offroad_goal_relay" output="screen"
        if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi&quot;')">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <remap from="~/input/goal" to="/planning/offroad_goal"/>
    <remap from="~/input/cancel" to="/planning/offroad_cancel"/>
  </node>

  <!-- mppi mode: /cmd_vel -> Control on the gate AUTO input (see auto_control_cmd_topic in control launch).
       NOTE: pair local_layer:=mppi with auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd at the top level. -->
  <node pkg="autoware_nav2_offroad" exec="cmd_vel_to_control_bridge_node" name="cmd_vel_to_control_bridge" output="screen"
        if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi&quot;')">
    <param from="$(var cmd_vel_bridge_param_file)"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="wheelbase_m" value="$(var vehicle_wheelbase_m)"/>
    <param name="initial_enabled" value="true"/>
    <remap from="~/input/cmd_vel" to="/cmd_vel"/>
    <remap from="~/output/control_cmd" to="/nav2_offroad/mppi/control_cmd"/>
    <remap from="~/output/gear_cmd" to="/planning/gear_cmd"/>
  </node>
```
Also update the `local_layer` description comment: mppi mode is now drivable when `auto_control_cmd_topic` is set (Task 4).

- [ ] **Step 3: Build + launch-parse both modes**

```bash
docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && source _install/setup.bash && \
  ros2 launch autoware_nav2_offroad nav2_offroad.launch.xml local_layer:=mppi --show-args >/dev/null && echo MPPI_PARSE_OK && \
  ros2 launch autoware_nav2_offroad nav2_offroad.launch.xml --show-args >/dev/null && echo BRIDGE_PARSE_OK'
```
Expected: both `_PARSE_OK` lines. Then a 12 s live bringup of `local_layer:=mppi` and `ros2 node list` must show `/offroad_goal_relay` and `/cmd_vel_to_control_bridge` (pkill everything after).

- [ ] **Step 4: Commit**

```bash
git add tier4_universe_launch/autoware_nav2_offroad/launch/nav2_offroad.launch.xml tier4_universe_launch/autoware_nav2_offroad/config/nav2_mppi_controller.param.yaml
git commit -m "feat(nav2_offroad): wire goal relay + cmd_vel bridge into mppi mode; forward-only v1 (vx_min 0)"
```

---

### Task 4: `auto_control_cmd_topic` arg threading (gate AUTO input)

**Files:**
- Modify: `tier4_universe_launch/tier4_control_launch/launch/control.launch.xml` (arg + the `input/auto/control_cmd` remap, currently hardcoded at ~line 99)
- Modify: `autoware_launch/launch/components/tier4_control_component.launch.xml` (pass-through, mirroring `auto_gear_cmd_topic`)
- Modify: `autoware_launch/launch/autoware.launch.xml` (top-level arg, mirroring `auto_gear_cmd_topic`)

**Interfaces:**
- Produces: launching the stack with `auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd` makes the gate consume the mppi bridge output; default `/control/trajectory_follower/control_cmd` = zero behavior change.

- [ ] **Step 1: control.launch.xml**

Next to the existing `auto_gear_cmd_topic` arg (~line 64):
```xml
  <arg name="auto_control_cmd_topic" default="/control/trajectory_follower/control_cmd"/>
```
Change the hardcoded remap (~line 99):
```xml
          <remap from="input/auto/control_cmd" to="$(var auto_control_cmd_topic)"/>
```

- [ ] **Step 2: Thread through the component + top level**

In `tier4_control_component.launch.xml` and `autoware.launch.xml`, add the arg declaration and pass-through **exactly where and how `auto_gear_cmd_topic` is declared/passed in each file** (grep `auto_gear_cmd_topic` in each; add the sibling line with default `/control/trajectory_follower/control_cmd`).

- [ ] **Step 3: Verify — default unchanged, override propagates**

```bash
docker exec autoware bash -lc 'cd /workspace/_mppi_drive && grep -n "auto_control_cmd_topic" \
  tier4_universe_launch/tier4_control_launch/launch/control.launch.xml \
  autoware_launch/launch/components/tier4_control_component.launch.xml \
  autoware_launch/launch/autoware.launch.xml'
```
Expected: arg present in all three files. Then rebuild the two launch packages (`--packages-select autoware_launch tier4_control_launch` with the isolated bases) and parse-check:
```bash
docker exec autoware bash -lc 'cd /workspace/_mppi_drive && source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && source _install/setup.bash && \
  ros2 launch tier4_control_launch control.launch.xml --show-args 2>/dev/null | grep -A1 auto_control_cmd_topic && echo THREAD_OK'
```
Expected: the arg shows with its default; `THREAD_OK`.

- [ ] **Step 4: Commit**

```bash
git add tier4_universe_launch/tier4_control_launch/launch/control.launch.xml autoware_launch/launch/components/tier4_control_component.launch.xml autoware_launch/launch/autoware.launch.xml
git commit -m "feat(control_launch): make the gate auto control-cmd input configurable (auto_control_cmd_topic)"
```

---

### Task 5: Docs + full-suite verification

**Files:**
- Modify: `tier4_universe_launch/autoware_nav2_offroad/BACKLOG.md` (item #11: mark goal relay + cmd_vel routing DONE; remaining = sim engage check, reverse follow-up, GPU/critic tuning)
- Modify: `tier4_universe_launch/autoware_nav2_offroad/README.md` (mppi drivable section)
- Modify: `tier4_universe_launch/autoware_nav2_offroad/TUNING.md` (one-line pointer: forward-only vx_min, where to re-enable reverse later)

**README section must state:**
- Run command pairing: `planning_simulator ... navigation_mode:=nav2_offroad local_layer:=mppi auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd occupancy_grid_source:=perception` (recommended costmap pairing).
- Safety model in mppi mode (spec §5): validator NOT in the loop; chain = ObstaclesCritic → bridge stale-cmd_vel watchdog → gate filter; abort = `/planning/offroad_cancel` (RViz "Activate ON-ROAD" + RTH cancel work unchanged).
- **Open sim-check caveat** (spec §6): AUTONOMOUS engage without `/planning/trajectory` must be verified in sim; `allow_autonomous_in_stopped: true` should allow standstill engage.
- Acceptance test: `scripts/offroad_demo_tour.py` under mppi mode.

- [ ] **Step 1: Write the three doc updates** (per the bullets above — keep BACKLOG #11's existing structure, append an "update 2026-07-02" block).

- [ ] **Step 2: Full package test suite**

Global Constraints test command (no `-R` filter). Expected: **119 tests, 0 failures** (baseline 114 + 5 ported bridge tests), plus both launch-parse checks from Tasks 3–4 still green.

- [ ] **Step 3: Commit**

```bash
git add tier4_universe_launch/autoware_nav2_offroad/BACKLOG.md tier4_universe_launch/autoware_nav2_offroad/README.md tier4_universe_launch/autoware_nav2_offroad/TUNING.md
git commit -m "docs(nav2_offroad): mppi drivable mode — README/BACKLOG/TUNING (routing, safety model, sim-check caveat)"
```

---

## Self-Review notes (spec coverage)

- Spec §3.1 goal relay → Task 2 (incl. rejection callback, startup NONE, latest-goal-wins preemption, cancel). §3.2 bridge port + `initial_enabled` + gear remap → Tasks 1 & 3. §3.3 launch/arg threading → Tasks 3 & 4 (three-file chain verified against the `auto_gear_cmd_topic` precedent). §4 forward-only → Task 3 Step 1. §5 safety model + §6 engage caveat → Task 5 README. §7 follow-ups stay out.
- Type consistency: relay result codes match the NavThroughPoses-bridge convention (0–4); bridge interfaces match the ported files verbatim.
- Not in this plan (by design): the end-to-end sim drive (`offroad_demo_tour.py` under mppi) — final acceptance, needs the full simulator; documented as the acceptance command in Task 5.
