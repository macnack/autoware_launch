# Tutorial: running off-road (Nav2) navigation

End-to-end: launch the stack, switch into off-road mode, send a goal, and drive.
Also lists what is still missing for a full deployment.

> Terminology: off-road is **not (yet) an `autoware_scenario_selector` scenario**.
> It runs as a parallel planner selected at runtime by `trajectory_mode_manager`.
> You "call" it by (1) putting the manager in `NAV2_OFFROAD` mode and (2) sending
> a goal on `/planning/offroad_goal`. Folding it into `scenario_selector` (so it
> becomes a first-class scenario with automatic, map-based selection) is the
> planned next step — see [MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md) §10.

## 0. Prerequisites

- **Nav2 installed** (not in stock Autoware): `nav2_planner`, `nav2_smoother`,
  `nav2_navfn_planner`, `nav2_smac_planner` (provides the heading-aware
  SmacPlannerHybrid), `nav2_lifecycle_manager`. They are declared in this
  package's `package.xml`, so `rosdep install --from-paths src --ignore-src -y`
  pulls them; or
  `apt install ros-$ROS_DISTRO-nav2-{planner,smoother,navfn-planner,smac-planner,lifecycle-manager}`.
- Workspace built and sourced: `colcon build` → `source install/setup.bash`.
- **Localization running** (an ego pose on `/localization/kinematic_state`). Without
  it the manager stays in `STANDBY` and only emits a safe-stop.
- A costmap source — `free` (default, no sensors) or `perception` (needs a LiDAR +
  Autoware perception; see [README](README.md) and `occupancy_grid_source`).

## 1. Launch the simulator in off-road mode

Off-road is the `OFFROAD` scenario of `autoware_scenario_selector`, enabled by
`navigation_mode:=nav2_offroad`:

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  navigation_mode:=nav2_offroad \
  initial_engage_state:=true \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

This brings up the planning_simulator plus the Nav2 off-road stack (planner +
smoother + bridge), wires the Nav2 bridge into `scenario_selector`, and uses the
**free occupancy map by default** (no sensors needed — ideal for sim). RViz opens
automatically.

> For the real perception occupancy grid add `occupancy_grid_source:=perception`
> — that needs a LiDAR/perception pipeline, which is **not** present in
> `planning_simulation`, so keep the default `free` in sim.

## 2. Set the initial pose

In RViz, click **2D Pose Estimate** and place the vehicle on the map so
localization publishes an ego pose on `/localization/kinematic_state`. With
`initial_engage_state:=true` the vehicle is auto-engaged (operation mode
AUTONOMOUS); otherwise engage via the RViz AutowareStatePanel / AD API.

## 3. Send an off-road goal — this activates OFFROAD

Publish a goal on the dedicated off-road topic, in the `map` frame (pick a point
on the map; with the free costmap any reachable point works):

```bash
ros2 topic pub --once /planning/offroad_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 30.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}"
```

`scenario_selector` switches to the **OFFROAD** scenario while the goal is set and
unreached. The Nav2 bridge plans a path (`ComputePathToPose` + `SmoothPath`) and
publishes a trajectory, which flows:

```
Nav2 bridge → scenario_selector(OFFROAD) → velocity_smoother → planning_validator → /planning/trajectory
```

The vehicle drives to the goal; on arrival (reached + stopped) `scenario_selector`
clears OFFROAD and falls back to lane/parking selection.

> No RViz goal tool is wired to `/planning/offroad_goal` yet — use the command
> above, or remap the RViz "2D Goal Pose" tool to that topic in your RViz config.

## 4. Verify

```bash
ros2 topic echo --once /planning/scenario_planning/scenario   # current_scenario: OffRoad
ros2 topic hz   /planning/trajectory                          # output flowing
ros2 topic echo --once /nav2_offroad/planning/trajectory      # Nav2 bridge output
```

In RViz, add a `Trajectory`/`Path` display for `/planning/trajectory` and
`/nav2_offroad/planning/trajectory`, and a `Map`/`OccupancyGrid` display for
`/nav2_offroad/costmap/occupancy_grid`. Record a run for offline review with
`ros2 launch autoware_nav2_offroad debug_record.launch.xml`. More in
[DEBUGGING.md](DEBUGGING.md).

## 5. Return to on-road

Send a normal on-road goal (RViz "2D Goal Pose" with the routing adaptor, or the
mission planner). Once the off-road goal is reached, `scenario_selector` resumes
`LANEDRIVING`/`PARKING` selection automatically.

## Legacy: standalone mode-manager path

The earlier standalone `trajectory_mode_manager` (a runtime `change_mode` service +
the RViz `OffroadModePanel` button) is **opt-in** and not used by the
`scenario_selector` flow above. To use it instead, launch `nav2_offroad.launch.xml`
with `launch_trajectory_mode_manager:=true` and set
`planning_validator_output_trajectory:=/planning/trajectory_pre_mux`. See
[MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md).

## What is still missing

| Gap | Impact | Status |
|-----|--------|--------|
| Nav2 not in stock Autoware | must `rosdep`/apt-install nav2 | provisioning step (see §0) |
| Not an `autoware_scenario_selector` scenario | no automatic map-based on/off-road transitions, no native HMI; invoked via the mode manager | planned (Option C) |
| No RViz goal tool for `/planning/offroad_goal` | goal set via CLI or a manual RViz tool remap | convenience gap |
| Global-planner-only | Nav2 global costmap + planner + smoother only — **no Nav2 local costmap / controller / recovery behaviors**, and Autoware's obstacle modules are off in off-road mode. Static-obstacle avoidance via the costmap only; no dynamic-obstacle avoidance | design limitation |
| Real costmap needs LiDAR perception | `perception` source requires the sensing/OGM pipeline | hardware/stack dependency |
| Tuning per vehicle | clearance, speed, guards, actuation limits | see [TUNING.md](TUNING.md) |

### Roadmap

1. Integrate OFFROAD into `autoware_scenario_selector` (first-class scenario,
   automatic transitions, HMI) — [MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md) §10.
2. RViz goal tool bound to `/planning/offroad_goal` (+ a shipped RViz config with
   the panel, markers, and goal tool).
3. Add a Nav2 local costmap + controller (or reuse Autoware obstacle avoidance) for
   dynamic-obstacle handling off-road.
