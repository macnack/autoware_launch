# MPPI recovery mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an experimental `local_layer:=mppi_recovery` off-road mode that wraps the existing MPPI drive stack in the standard Nav2 robustness layer (behavior_server + a forward-only recovery BT + a loosened Hybrid-planner tolerance), so a transient planner failure recovers instead of aborting the goal.

**Architecture:** A third `local_layer` value alongside `bridge`/`mppi`. It launches everything `mppi` launches plus a lifecycle-managed `behavior_server`, points `bt_navigator` at recovery behavior trees (RecoveryNode → retry with ClearEntireCostmap + Wait), and loosens the `SmacPlannerHybrid` goal tolerance via launch `<let>` value overrides. The existing `mppi` and `bridge` modes are untouched.

**Tech Stack:** ROS 2 Humble, Nav2 (`nav2_bt_navigator`, `nav2_behaviors`, `nav2_smac_planner`, `nav2_mppi_controller`), BehaviorTree.CPP v3, ROS 2 XML launch, ament/colcon.

## Global Constraints

- Build and test ONLY inside the `autoware` container, never on the host.
- `git add` only the specific files listed in each task's commit; never `git add -A`.
- Commit only when a task's steps are complete; end commit messages with `Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>`.
- The `mppi` and `bridge` modes must remain byte-for-byte behaviorally unchanged.
- Forward-only: no `spin`, no `backup`, no reverse. `DUBIN` model and `vx_min: 0.0` retained.
- All paths below are relative to the package root `tier4_universe_launch/autoware_nav2_offroad/` in the `_mppi_drive` worktree (branch `feat/offroad-mppi-drive`).
- Container ROS env for any runtime command: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///workspace/.devcontainer/cyclonedds_config.xml` then source `/opt/ros/humble/setup.bash`, `/workspace/install/setup.bash`, and the package build overlay.

---

### Task 1: behavior_server parameter config

**Files:**
- Create: `config/nav2_behavior_server.param.yaml`

**Interfaces:**
- Produces: a `behavior_server` param file exposing the `wait` and `drive_on_heading` behavior action servers (`/wait`, `/drive_on_heading`) that Task 2's recovery BT and Task 3's launch consume. Node name at launch: `behavior_server`.

- [ ] **Step 1: Write the config file**

```yaml
# behavior_server (nav2_behaviors) for local_layer:=mppi_recovery (EXPERIMENTAL).
# Forward-only recovery set: wait + drive_on_heading. NO spin/backup (a forward-only
# Ackermann off-road vehicle cannot rotate in place or reverse). Provides the recovery
# action servers the mppi_recovery behavior trees call between planner retries.
behavior_server:
  ros__parameters:
    use_sim_time: false
    local_costmap_topic: /local_costmap/costmap_raw
    local_footprint_topic: /local_costmap/published_footprint
    global_costmap_topic: /global_costmap/costmap_raw
    global_footprint_topic: /global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["wait", "drive_on_heading"]
    wait:
      plugin: "nav2_behaviors::Wait"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.2
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.0
    min_rotational_vel: 0.0
    rotational_acc_lim: 0.0
```

- [ ] **Step 2: Verify YAML lints clean**

Run (in container, package root): `pre-commit run yamllint --files config/nav2_behavior_server.param.yaml`
Expected: PASS (or the hook reports no yamllint errors on this file).

- [ ] **Step 3: Verify it installs**

The `config/` directory is already installed by `CMakeLists.txt` (`install(DIRECTORY config launch ...)`), so no CMake change is needed. Confirm the file is under `config/`:
Run: `ls config/nav2_behavior_server.param.yaml`
Expected: the path prints.

- [ ] **Step 4: Commit**

```bash
git add config/nav2_behavior_server.param.yaml
git commit -m "feat(nav2_offroad): behavior_server config for mppi_recovery (wait + drive_on_heading, no spin/backup)"
```

---

### Task 2: Forward-only recovery behavior trees

**Files:**
- Create: `config/behavior_trees/navigate_to_pose_recovery_offroad.xml`
- Create: `config/behavior_trees/navigate_through_poses_recovery_offroad.xml`

**Interfaces:**
- Consumes: the `wait` action server from Task 1; the `GridBased` planner and `FollowPath` controller already configured in `nav2_offroad.param.yaml` / `nav2_mppi_controller.param.yaml`.
- Produces: two BT xml files referenced by Task 3's `bt_navigator` `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` for `mppi_recovery`.

- [ ] **Step 1: Write the NavigateToPose recovery BT**

Create `config/behavior_trees/navigate_to_pose_recovery_offroad.xml`:

```xml
<!--
  Forward-only NavigateToPose recovery BT for local_layer:=mppi_recovery (EXPERIMENTAL).
  RecoveryNode retries the plan+follow loop; on failure it clears both costmaps and waits,
  then retries. NO Spin/BackUp (forward-only Ackermann vehicle). BT.CPP v3 (ROS 2 Humble).
  planner_id=GridBased and controller_id=FollowPath match nav2_offroad.param.yaml /
  nav2_mppi_controller.param.yaml.
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
        <Wait wait_duration="2.0"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

- [ ] **Step 2: Write the NavigateThroughPoses recovery BT**

Create `config/behavior_trees/navigate_through_poses_recovery_offroad.xml` (bt_navigator instantiates both navigators, so this must also be recovery-server-compatible):

```xml
<!--
  Forward-only NavigateThroughPoses recovery BT for mppi_recovery (EXPERIMENTAL).
  Same structure as the to-pose recovery BT but ComputePathThroughPoses(goals).
  No Spin/BackUp. BT.CPP v3 (ROS 2 Humble).
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
        <Wait wait_duration="2.0"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

- [ ] **Step 3: Verify both XML files are well-formed**

Run: `xmllint --noout config/behavior_trees/navigate_to_pose_recovery_offroad.xml config/behavior_trees/navigate_through_poses_recovery_offroad.xml`
Expected: no output (well-formed). If `xmllint` is absent, run `python3 -c "import xml.dom.minidom as m; m.parse('config/behavior_trees/navigate_to_pose_recovery_offroad.xml'); m.parse('config/behavior_trees/navigate_through_poses_recovery_offroad.xml'); print('ok')"` → prints `ok`.

- [ ] **Step 4: Verify no forbidden nodes present**

Run: `grep -iE "Spin|BackUp|AssistedTeleop" config/behavior_trees/navigate_to_pose_recovery_offroad.xml config/behavior_trees/navigate_through_poses_recovery_offroad.xml || echo "clean (no spin/backup)"`
Expected: prints `clean (no spin/backup)`.

- [ ] **Step 5: Commit**

```bash
git add config/behavior_trees/navigate_to_pose_recovery_offroad.xml config/behavior_trees/navigate_through_poses_recovery_offroad.xml
git commit -m "feat(nav2_offroad): forward-only recovery BTs for mppi_recovery (retry + clear-costmap + wait)"
```

---

### Task 3: Launch integration — `local_layer:=mppi_recovery`

**Files:**
- Modify: `launch/nav2_offroad.launch.xml`

**Interfaces:**
- Consumes: `config/nav2_behavior_server.param.yaml` (Task 1) and the two recovery BTs (Task 2).
- Produces: a working `local_layer:=mppi_recovery` mode. Node set = the `mppi` set plus `behavior_server`; `bt_navigator` uses the recovery BTs; `SmacPlannerHybrid` uses the loosened tolerance.

Note: ROS 2 Humble `launch_xml` does not allow `if`/`unless` on `<param>`, so all mode-conditional values are selected with `<let>` variables and applied as `<param name= value=>`. Conditions broaden from `==mppi` to `in [mppi, mppi_recovery]` where a node is shared.

- [ ] **Step 1: Broaden the `local_layer` arg description**

In `launch/nav2_offroad.launch.xml`, change the `local_layer` arg (currently `bridge | mppi`) description to include the new value. Find:

```xml
  <arg name="local_layer" default="bridge" description="bridge | mppi"/>
```

Replace with:

```xml
  <arg name="local_layer" default="bridge" description="bridge | mppi | mppi_recovery (mppi_recovery = EXPERIMENTAL: mppi + behavior_server + recovery BTs + loosened planner)"/>
```

- [ ] **Step 2: Extend the `lifecycle_nodes` selection to three cases**

Replace the existing two `<let name="lifecycle_nodes" ...>` lines (the `unless`/`if` pair on `==mppi`) with three mutually-exclusive lets:

```xml
  <!-- lifecycle node list depends on the local layer -->
  <let name="lifecycle_nodes" value="[planner_server, smoother_server]"
       unless="$(eval '&quot;$(var local_layer)&quot; in [&quot;mppi&quot;,&quot;mppi_recovery&quot;]')"/>
  <let name="lifecycle_nodes" value="[planner_server, smoother_server, controller_server, bt_navigator]"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi&quot;')"/>
  <let name="lifecycle_nodes" value="[planner_server, smoother_server, controller_server, bt_navigator, behavior_server]"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
```

- [ ] **Step 3: Add `<let>` selectors for the BT xml paths and planner overrides**

Immediately after the `lifecycle_nodes` lets, add:

```xml
  <!-- mppi_recovery selects recovery BTs + a loosened Hybrid tolerance; mppi keeps the
       recovery-free BTs and stock tolerance. (launch_xml forbids conditional <param>, so
       select via <let>.) -->
  <let name="bt_to_pose_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_to_pose_no_recovery.xml"
       unless="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="bt_to_pose_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_to_pose_recovery_offroad.xml"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="bt_through_poses_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_through_poses_no_recovery.xml"
       unless="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="bt_through_poses_xml"
       value="$(find-pkg-share autoware_nav2_offroad)/config/behavior_trees/navigate_through_poses_recovery_offroad.xml"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="planner_goal_tol" value="0.25"
       unless="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="planner_goal_tol" value="0.5"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="planner_approach_iters" value="1000"
       unless="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
  <let name="planner_approach_iters" value="8000"
       if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')"/>
```

- [ ] **Step 4: Broaden the shared mppi node conditions**

For each of `controller_server`, `offroad_goal_relay`, and `cmd_vel_to_control_bridge`, change the node's condition from `if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi&quot;')"` to:

```xml
        if="$(eval '&quot;$(var local_layer)&quot; in [&quot;mppi&quot;,&quot;mppi_recovery&quot;]')"
```

(Three nodes; the `bt_navigator` node is handled in Step 5. Do NOT touch the `bridge`-only `nav2_path_to_trajectory_bridge` node.)

- [ ] **Step 5: Rewire `bt_navigator` to use the selected BT xmls + broaden its condition**

Replace the whole `bt_navigator` `<node>` block with (condition broadened; BT params now use the `<let>` vars from Step 3):

```xml
  <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator" output="screen"
        if="$(eval '&quot;$(var local_layer)&quot; in [&quot;mppi&quot;,&quot;mppi_recovery&quot;]')">
    <param from="$(var bt_nav_param_file)"/>
    <!-- BT selection is mode-dependent (see the bt_*_xml lets above): mppi uses the
         recovery-free BTs; mppi_recovery uses the recovery BTs that need behavior_server. -->
    <param name="default_nav_to_pose_bt_xml" value="$(var bt_to_pose_xml)"/>
    <param name="default_nav_through_poses_bt_xml" value="$(var bt_through_poses_xml)"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
```

- [ ] **Step 6: Add the `behavior_server` node (mppi_recovery only)**

Immediately after the `bt_navigator` node, add:

```xml
  <!-- mppi_recovery only: recovery action servers (wait, drive_on_heading) for the recovery BTs. -->
  <node pkg="nav2_behaviors" exec="behavior_server" name="behavior_server" output="screen"
        if="$(eval '&quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot;')">
    <param from="$(find-pkg-share autoware_nav2_offroad)/config/nav2_behavior_server.param.yaml"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
```

- [ ] **Step 7: Apply the planner tolerance overrides to both planner_server nodes**

On BOTH `planner_server` `<node>` blocks (the `global_planner!=lattice` one and the `global_planner==lattice` one), add these two params after the existing `<param>` lines:

```xml
    <param name="GridBased.tolerance" value="$(var planner_goal_tol)"/>
    <param name="GridBased.max_on_approach_iterations" value="$(var planner_approach_iters)"/>
```

- [ ] **Step 8: Verify the launch parses for all three modes**

Build first (container, workspace root):
Run: `colcon build --symlink-install --packages-select autoware_nav2_offroad --base-paths /workspace/src --build-base /workspace/_mppi_drive/_build --install-base /workspace/_mppi_drive/_install`
Expected: `Finished <<< autoware_nav2_offroad` with no errors.

Then parse-check each mode (source `/opt/ros/humble`, `/workspace/install`, `/workspace/_mppi_drive/_install`):
Run: `for m in bridge mppi mppi_recovery; do echo "== $m =="; ros2 launch autoware_nav2_offroad nav2_offroad.launch.xml local_layer:=$m --show-args >/dev/null 2>/tmp/parse_$m.log && echo "parsed OK" || { echo "PARSE ERROR"; cat /tmp/parse_$m.log; }; done`
Expected: each mode prints `parsed OK` (no XML/eval errors). If `--show-args` on the standalone file complains about missing required args, instead assert the file loads without a frontend/eval error using: `python3 -c "from launch.launch_description_sources import get_launch_description_from_any_launch_file as L; L('launch/nav2_offroad.launch.xml'); print('mppi_recovery frontend ok')"` (run once; a syntactically bad eval/let raises here).

- [ ] **Step 9: Verify mppi/bridge node sets are unchanged and mppi_recovery adds behavior_server**

Run: `grep -c "behavior_server" launch/nav2_offroad.launch.xml`
Expected: `2` (the lifecycle list entry + the node), i.e. behavior_server only appears for mppi_recovery. Manually confirm the three broadened conditions (`controller_server`, `offroad_goal_relay`, `cmd_vel_to_control_bridge`) and `bt_navigator` now read `in [mppi, mppi_recovery]`, and no `bridge`-only node changed.

- [ ] **Step 10: Commit**

```bash
git add launch/nav2_offroad.launch.xml
git commit -m "feat(nav2_offroad): add EXPERIMENTAL local_layer:=mppi_recovery (behavior_server + recovery BTs + loosened Hybrid tolerance)"
```

---

### Task 4: Documentation — README + BACKLOG (EXPERIMENTAL)

**Files:**
- Modify: `README.md`
- Modify: `BACKLOG.md` (if present; if absent, add the note to `README.md` only)

**Interfaces:**
- Consumes: nothing. Documents the mode delivered in Tasks 1–3.

- [ ] **Step 1: Add the `mppi_recovery` row to the README `local_layer` table**

In `README.md`, find the `local_layer` args table row (the one describing `bridge | mppi`). Update the value cell to `bridge \| mppi \| mppi_recovery` and append to the description:

```
`mppi_recovery` (**EXPERIMENTAL**): `mppi` plus the standard Nav2 recovery layer — `behavior_server` (`wait`, `drive_on_heading`; no spin/backup, forward-only) + recovery behavior trees (RecoveryNode: retry + clear-costmap + wait) + a loosened `SmacPlannerHybrid` goal tolerance (0.5 m). Recovers from transient planner failures instead of aborting the goal. Pair with `auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd` exactly like `mppi`.
```

- [ ] **Step 2: Add a short "Experimental: mppi_recovery" subsection to the README**

Add a subsection near the `local_layer:=mppi` docs:

```markdown
### Experimental: `local_layer:=mppi_recovery`

Same drive chain as `mppi` (goal → relay → bt_navigator → planner → MPPI → bridge → gate)
but with the standard Nav2 robustness layer so a single planner failure recovers instead of
aborting: a lifecycle-managed `behavior_server` (forward-only `wait` + `drive_on_heading`),
recovery behavior trees that retry `ComputePathToPose` with `ClearEntireCostmap` + `Wait`
between attempts, and a loosened `SmacPlannerHybrid` goal tolerance (0.5 m, more approach
iterations). Forward-only; no reverse, no spin/backup. Marked experimental until the sim
acceptance (below) is validated on a clean workspace.
```

- [ ] **Step 3: Add a BACKLOG note for the follow-ups**

If `BACKLOG.md` exists, append:

```markdown
- `mppi_recovery` follow-ups: (a) reverse variant (REEDS_SHEPP planner + MPPI reverse + backup recovery) to fix exact-pose alignment; (b) MPPI weaving/tracking tuning to reduce how often recovery triggers; (c) promote `mppi_recovery` to default once validated.
```

If `BACKLOG.md` does not exist, skip this step (the README subsection already records the experimental status).

- [ ] **Step 4: Verify markdown lints**

Run: `pre-commit run markdownlint --files README.md` (and `BACKLOG.md` if modified)
Expected: PASS (or no markdownlint errors on the changed files).

- [ ] **Step 5: Commit**

```bash
git add README.md BACKLOG.md
git commit -m "docs(nav2_offroad): document EXPERIMENTAL mppi_recovery mode + backlog follow-ups"
```

(If `BACKLOG.md` was not modified, drop it from the `git add`.)

---

### Task 5: Runtime acceptance — behavior_server activates + the 22 m goal reaches

This task has no unit test (it needs the full planning simulator, which is not available in CI). It is a documented, repeatable runtime procedure run inside the `autoware` container. Record the outcome in the commit message of Task 4 or as a note; do not mark the plan complete until it passes on a clean workspace.

**Files:**
- None (verification only). Optionally Create: `docs/superpowers/mppi_recovery_acceptance.md` to record the procedure + result.

- [ ] **Step 1: Launch the planning simulator in mppi_recovery mode**

In the container (with the env from Global Constraints), run the standard offroad planning_simulator launch with `local_layer:=mppi_recovery auto_control_cmd_topic:=/nav2_offroad/mppi/control_cmd occupancy_grid_source:=free`. Let it come up (~40 s).

- [ ] **Step 2: Verify behavior_server activates**

Run: `ros2 lifecycle get /behavior_server`
Expected: `active [3]`. Also `ros2 node list | grep behavior_server` prints `/behavior_server`, and `ros2 action list | grep -E "/wait|/drive_on_heading"` shows both action servers.

- [ ] **Step 3: Verify all five managed nodes are active**

Run: `for n in planner_server smoother_server controller_server bt_navigator behavior_server; do echo -n "$n: "; ros2 lifecycle get /$n; done`
Expected: each prints `active [3]`.

- [ ] **Step 4: Run the acceptance goal (the one that aborted in mppi mode)**

Set the vehicle to a known start and publish an off-road goal ~22 m straight ahead, aligned to heading (the exact scenario that aborted in `mppi`). Drive it via the same LOCAL-mode path used during validation (or, on a clean workspace with a green diagnostic graph, engage AUTONOMOUS normally). Watch `/offroad_goal_relay/result`.
Expected: the goal reaches (`result` = SUCCEEDED) rather than ABORTED — i.e. any transient `planner_server` "exceeded maximum iterations" is followed by a ClearEntireCostmap + Wait retry and the drive continues, instead of the goal failing.

- [ ] **Step 5: Verify mppi mode still works unchanged (regression)**

Relaunch with `local_layer:=mppi`. Confirm `ros2 node list | grep behavior_server` prints nothing (behavior_server not launched), `bt_navigator` reaches `active`, and the recovery-free BT is in use (`ros2 param get /bt_navigator default_nav_to_pose_bt_xml` ends in `navigate_to_pose_no_recovery.xml`).
Expected: as stated — `mppi` unchanged.

- [ ] **Step 6: Record the result**

Note pass/fail (and any tuning of `number_of_retries` / `Wait` / `tolerance` needed) in the acceptance doc or the PR description. If the goal still aborts, the follow-up is MPPI weaving tuning and/or the reverse variant (per the spec's out-of-scope list), not this mode's plumbing.

---

## Self-Review

**1. Spec coverage:**
- §4.1 launch (mppi_recovery value, lifecycle_nodes, broadened conditions, BT selection, planner override) → Task 3 (all steps). ✔
- §4.2 behavior_server config → Task 1. ✔
- §4.3 recovery BTs (to-pose + through-poses) → Task 2. ✔
- §4.4 loosened planner tolerance → Task 3 Steps 3, 7 (realized as `<let>` value overrides rather than a separate overlay file, because launch_xml forbids conditional `<param from>`; equivalent effect, `mppi`/`bridge` unaffected). ✔
- §4.5 goal checker consistency → planner tolerance loosened to 0.5 m matches the existing controller `goal_checker.xy_goal_tolerance: 0.5` in `nav2_mppi_controller.param.yaml`, so no override is required; documented here so it isn't a silent gap. ✔
- §5 forward-only / no spin-backup / experimental label → Tasks 1, 2, 4. ✔
- §6 testing (launch-parse, lifecycle, regression, sim acceptance) → Task 3 Steps 8–9, Task 5. ✔

**2. Placeholder scan:** No TBD/TODO; every new file's full content is inline; every launch edit shows exact before/after XML. ✔

**3. Type/name consistency:** BT paths `navigate_to_pose_recovery_offroad.xml` / `navigate_through_poses_recovery_offroad.xml` and the `<let>` names `bt_to_pose_xml` / `bt_through_poses_xml` / `planner_goal_tol` / `planner_approach_iters` are used identically across Task 2 and Task 3. Node name `behavior_server` and its param file `config/nav2_behavior_server.param.yaml` match across Tasks 1 and 3. `GridBased` / `FollowPath` ids match the existing configs. ✔

> **Deviation from spec §4.4 (noted):** the spec named a `nav2_offroad_recovery.param.yaml` overlay file; the plan instead uses `<let>` value overrides for the two planner params because ROS 2 Humble `launch_xml` cannot apply a `<param from="...">` conditionally on a single node without splitting the planner nodes (which are already split hybrid/lattice). Same effect, fewer node variants, `mppi`/`bridge` untouched.
