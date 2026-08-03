# RPP tracking controller (`local_controller:=rpp`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `local_controller:=rpp` so the off-road drive stack runs the classical split pipeline — Smac global plan → RegulatedPurePursuit pure tracking (with cost-regulated obstacle slowdown) → the existing acceleration bridge — with MPPI retained as the default/experimental alternative.

**Architecture:** Config-only + launch: a new RPP controller_server param file (same node/structure as the MPPI file, same goal checker, same 24×24 local costmap) selected by a `<let>` cascade, plus RPP forward/reverse overlays keyed to the existing `allow_reverse` flag (`allow_reversing`). No C++ — RPP publishes `/cmd_vel` through the same controller_server, so the bridge, GearArbiter, relay, BTs, and behavior_server are reused untouched.

**Tech Stack:** ROS 2 Humble, `nav2_regulated_pure_pursuit_controller` 1.1.20 (verified installed, `allow_reversing` supported), XML launch let-cascades.

## Global Constraints

- Build and test ONLY inside the `autoware` container, never on the host.
- `git add` only the files listed per task; never `git add -A`.
- Commit messages end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Default behavior (`local_controller` omitted → `mppi`) must be byte-for-byte unchanged; `bridge` mode untouched.
- Controller/reverse selection is ALWAYS a `<let>`-chosen FILE — never a string `<param value>` override of a typed param (known silent no-op).
- Paths relative to `tier4_universe_launch/autoware_nav2_offroad/` in the `_mppi_drive` worktree (branch `feat/offroad-mppi-drive`).
- Container build: `colcon build --symlink-install --packages-select autoware_nav2_offroad --base-paths /workspace/_mppi_drive/tier4_universe_launch --build-base /workspace/_mppi_drive/_build --install-base /workspace/_mppi_drive/_install`
- Runtime env for live commands: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///workspace/.devcontainer/cyclonedds_config.xml`, source `/opt/ros/humble`, `/workspace/install`, `/workspace/_mppi_drive/_install`, and restart the ros2 daemon before graph queries.

---

### Task 1: RPP config files

**Files:**
- Create: `config/nav2_rpp_controller.param.yaml`
- Create: `config/nav2_rpp_forward.param.yaml`
- Create: `config/nav2_rpp_reverse.param.yaml`

**Interfaces:**
- Produces: three yaml files whose exact paths Task 2's launch lets reference. The controller file keeps node name `controller_server`, plugin id `FollowPath`, the goal checker values (`xy 1.5` / `yaw 3.15`), and the same `local_costmap` section as `nav2_mppi_controller.param.yaml`, so the swap is purely a file swap.

- [ ] **Step 1: Write `config/nav2_rpp_controller.param.yaml`**

```yaml
# RegulatedPurePursuit tracking controller for the off-road local layer
# (local_controller:=rpp). The classical split pipeline: SmacPlannerHybrid plans a
# kinematically feasible path; RPP TRACKS it exactly (geometric lookahead chase),
# regulating speed on curvature and obstacle proximity (cost-regulated scaling =
# the local safety layer). Deterministic, ~5 knobs — the alternative to MPPI's
# sampling optimizer (which remains available via local_controller:=mppi).
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    controller_plugins: ["FollowPath"]
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 1.5      # position goal; small overshoot still counts
      yaw_goal_tolerance: 3.15    # ignore final heading (Ackermann can't align a pose)
      stateful: true
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 2.5
      lookahead_dist: 4.0
      use_velocity_scaled_lookahead_dist: true
      min_lookahead_dist: 2.0
      max_lookahead_dist: 6.0
      lookahead_time: 1.5
      transform_tolerance: 0.1
      min_approach_linear_velocity: 0.3
      approach_velocity_scaling_dist: 5.0   # decelerate into the goal
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true          # curvature slowdown
      use_cost_regulated_linear_velocity_scaling: true     # obstacle-proximity slowdown
      regulated_linear_scaling_min_radius: 3.5   # matches planner minimum_turning_radius
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: false   # Ackermann cannot rotate in place
      allow_reversing: false         # flipped by the reverse overlay
      max_robot_pose_search_dist: 10.0

# Same rolling local costmap as the MPPI file — RPP's cost-regulated slowdown reads it.
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 24
      height: 24
      resolution: 0.5
      robot_radius: 1.5
      track_unknown_space: false
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_topic: /nav2_offroad/costmap/occupancy_grid
        map_subscribe_transient_local: true
        subscribe_to_updates: false
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 2.0
      always_send_full_costmap: true
```

- [ ] **Step 2: Write the RPP motion overlays**

`config/nav2_rpp_forward.param.yaml`:

```yaml
# RPP motion overlay, forward-only (default): restates allow_reversing false.
controller_server:
  ros__parameters:
    FollowPath:
      allow_reversing: false
```

`config/nav2_rpp_reverse.param.yaml`:

```yaml
# RPP motion overlay for allow_reverse:=true: RPP natively detects direction
# cusps in the REEDS_SHEPP path and drives reverse segments backwards; the
# bridge's GearArbiter turns the sign into DRIVE/REVERSE with stop-and-shift.
controller_server:
  ros__parameters:
    FollowPath:
      allow_reversing: true
```

- [ ] **Step 3: Verify yaml parses**

Run (package root):
`python3 -c "import yaml; [yaml.safe_load(open(f)) for f in ['config/nav2_rpp_controller.param.yaml','config/nav2_rpp_forward.param.yaml','config/nav2_rpp_reverse.param.yaml']]; print('yaml ok')"`
Expected: `yaml ok`.

- [ ] **Step 4: Commit**

```bash
git add config/nav2_rpp_controller.param.yaml config/nav2_rpp_forward.param.yaml config/nav2_rpp_reverse.param.yaml
git commit -m "feat(nav2_offroad): RPP tracking-controller configs (split-pipeline local layer)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Launch integration — `local_controller`

**Files:**
- Modify: `launch/nav2_offroad.launch.xml`

**Interfaces:**
- Consumes: the three files from Task 1.
- Produces: launch arg `local_controller` (`mppi` default | `rpp`); lets `controller_param_file` and the now controller-aware `mppi_motion_overlay` (renamed `controller_motion_overlay`).

- [ ] **Step 1: Add the arg**

After the `allow_reverse` arg:

```xml
  <arg name="local_controller" default="mppi"
       description="mppi | rpp — FollowPath controller for local_layer mppi/mppi_recovery. rpp = RegulatedPurePursuit pure tracking (split pipeline: Smac plans, RPP tracks, cost-regulated slowdown is the local safety layer)."/>
```

- [ ] **Step 2: Controller file cascade**

Immediately after the `reverse_active` let:

```xml
  <!-- FollowPath controller selection: a <let>-chosen FILE (never string param overrides). -->
  <let name="controller_param_file"
       value="$(var mppi_param_file)"/>
  <let name="controller_param_file"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_rpp_controller.param.yaml"
       if="$(eval '&quot;$(var local_controller)&quot;==&quot;rpp&quot;')"/>
```

- [ ] **Step 3: Make the motion-overlay cascade controller-aware (2-D)**

Replace the existing `mppi_motion_overlay` lets (the pair added for allow_reverse) with a four-step cascade under a new name (later `<let>` wins — ordering encodes priority):

```xml
  <let name="controller_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_mppi_forward.param.yaml"/>
  <let name="controller_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_mppi_reverse.param.yaml"
       if="$(var reverse_active)"/>
  <let name="controller_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_rpp_forward.param.yaml"
       if="$(eval '&quot;$(var local_controller)&quot;==&quot;rpp&quot;')"/>
  <let name="controller_motion_overlay"
       value="$(find-pkg-share autoware_nav2_offroad)/config/nav2_rpp_reverse.param.yaml"
       if="$(eval '&quot;$(var local_controller)&quot;==&quot;rpp&quot; and $(var reverse_active)')"/>
```

NOTE on the last condition: `$(var reverse_active)` substitutes the eval-produced boolean text (`True`/`False`), which is a valid Python literal inside the outer eval. Verify in Step 5; if the frontend rejects it, use the expanded form
`if="$(eval '&quot;$(var local_controller)&quot;==&quot;rpp&quot; and &quot;$(var local_layer)&quot;==&quot;mppi_recovery&quot; and &quot;$(var allow_reverse)&quot;==&quot;true&quot;')"`.

- [ ] **Step 4: Point the controller node at the selected files**

In the `controller_server` node, replace
`<param from="$(var mppi_param_file)"/>` with `<param from="$(var controller_param_file)"/>`
and `<param from="$(var mppi_motion_overlay)"/>` with `<param from="$(var controller_motion_overlay)"/>`.

- [ ] **Step 5: Verify + build + parse the matrix**

Run: `xmllint --noout launch/nav2_offroad.launch.xml` → OK.
Build (container). Then parse-check:

```bash
for args in "local_layer:=mppi" "local_layer:=mppi local_controller:=rpp" \
            "local_layer:=mppi_recovery" "local_layer:=mppi_recovery local_controller:=rpp" \
            "local_layer:=mppi_recovery local_controller:=rpp allow_reverse:=true" \
            "local_layer:=bridge local_controller:=rpp"; do
  echo -n "  [$args] "; timeout 30 ros2 launch autoware_nav2_offroad nav2_offroad.launch.xml $args --show-args >/dev/null 2>&1 && echo OK || echo ERR
done
```
Expected: all six `OK`.

- [ ] **Step 6: Commit**

```bash
git add launch/nav2_offroad.launch.xml
git commit -m "feat(nav2_offroad): local_controller arg — RPP tracking controller selection (mppi default)

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Documentation

**Files:**
- Modify: `README.md` (args table + subsection)
- Modify: `BACKLOG.md` (follow-ups)

- [ ] **Step 1: README args table row** (after `allow_reverse`)

```markdown
| `local_controller` | `mppi` \| `rpp` | `mppi` | FollowPath controller for `local_layer` `mppi`/`mppi_recovery`. `rpp`: RegulatedPurePursuit **pure tracking** — the classical split pipeline (Smac plans, RPP tracks exactly, cost-regulated slowdown is the local safety layer). Deterministic, few knobs; recommended pairing `local_layer:=mppi_recovery local_controller:=rpp`. Reverse via `allow_reverse:=true` (`allow_reversing`). |
```

- [ ] **Step 2: README subsection** (after the Reverse subsection)

```markdown
#### Tracking controller (`local_controller:=rpp`)

MPPI is a sampling optimizer responsible for path following, avoidance, speed and
direction at once — powerful but tuning-heavy (weave/goal-miss/reverse-lock issues
under low-speed off-road conditions). `local_controller:=rpp` swaps in
RegulatedPurePursuit as a **pure tracking controller**: the Smac path (already
kinematically feasible) is tracked geometrically; speed is regulated by curvature,
obstacle proximity (cost-regulated scaling) and goal approach. Same
`controller_server`, `/cmd_vel`, bridge (acceleration + gear sequencing), recovery
BTs and gate routing. MPPI remains the default/experimental alternative.
```

- [ ] **Step 3: BACKLOG note** (append to the `mppi_recovery` follow-ups bullet)

```markdown
  (d) side-by-side `local_controller` mppi-vs-rpp validation; promote the winner to default.
```

- [ ] **Step 4: Commit**

```bash
git add README.md BACKLOG.md
git commit -m "docs(nav2_offroad): document local_controller:=rpp split pipeline

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: Runtime acceptance — the same two goals, on RPP

Requires a user relaunch (`task_reverse.sh` + `local_controller:=rpp`).

- [ ] **Step 1:** Add `local_controller:=rpp` to `task_reverse.sh` (or a `task_rpp.sh` copy); user relaunches.
- [ ] **Step 2:** Live-verify: `ros2 param get /controller_server FollowPath.plugin` →
  `...RegulatedPurePursuitController`; `FollowPath.allow_reversing` → `true`;
  `FollowPath.desired_linear_vel` → `2.5`.
- [ ] **Step 3:** Passthrough relay + LOCAL mode + reset pose (existing `acceptance2.sh` flow):
  **Test A** 15 m forward goal → drives straight in, decelerates near goal, REACHED, no
  weaving/orbiting. **Test B** 8 m behind goal → reverses (gear REVERSE) and REACHED.
- [ ] **Step 4:** Regression: relaunch default (no `local_controller`) →
  `FollowPath.plugin` is the MPPI controller; behavior as before.
- [ ] **Step 5:** Record results; tune only if a test fails (lookahead / desired_linear_vel
  first).

---

## Self-Review

**1. Spec coverage:** §4.1 RPP config → Task 1 Step 1. §4.2 overlays → Task 1 Step 2. §4.3 launch (arg, controller cascade, 2-D overlay cascade, node rewiring) → Task 2. §4.4 docs → Task 3. §6 testing (parse matrix incl. `bridge` ignoring args, regression, sim acceptance A/B, live param verify) → Task 2 Step 5 + Task 4. ✔

**2. Placeholder scan:** none — full file contents and exact XML inline; the one conditional-syntax risk (nested `$(var reverse_active)` in eval) carries an explicit verified fallback. ✔

**3. Name consistency:** `controller_param_file` / `controller_motion_overlay` used identically in Task 2 Steps 2–4; file paths match Task 1 exactly; `FollowPath` plugin id consistent with the BTs' `controller_id="FollowPath"`. ✔
