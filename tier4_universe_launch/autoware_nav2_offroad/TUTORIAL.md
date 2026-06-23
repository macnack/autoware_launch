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
  `nav2_navfn_planner`, `nav2_lifecycle_manager`. They are declared in this
  package's `package.xml`, so `rosdep install --from-paths src --ignore-src -y`
  pulls them; or `apt install ros-$ROS_DISTRO-nav2-{planner,smoother,navfn-planner,lifecycle-manager}`.
- Workspace built and sourced: `colcon build` → `source install/setup.bash`.
- **Localization running** (an ego pose on `/localization/kinematic_state`). Without
  it the manager stays in `STANDBY` and only emits a safe-stop.
- A costmap source — `free` (default, no sensors) or `perception` (needs a LiDAR +
  Autoware perception; see [README](README.md) and `occupancy_grid_source`).

## 1. Launch

**Option A — quick (simulator, runtime switch).** Both stacks run; switch at runtime.

```bash
ros2 launch autoware_nav2_offroad planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

**Option B — launch-time off-road mode.** Also disables road-only nodes and swaps
the diagnostics/component-monitor configs (requires the launch-time integration on
this branch):

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  navigation_mode:=nav2_offroad \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

To use the real perception occupancy grid instead of the free map, add
`occupancy_grid_source:=perception` to the `nav2_offroad` launch (or set it via the
wrapper).

## 2. Switch into off-road mode

The manager starts in `AW_PLANNING` (param `mode_on_startup`). Switch it to
off-road by any of:

- **RViz panel** — add *Panels → Add New Panel → OffroadModePanel*, click
  **Activate OFF-ROAD (Nav2)**.
- **Service**:

  ```bash
  ros2 service call /nav2_offroad/mode_manager/change_mode \
    autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode \
    "{target_mode: NAV2_OFFROAD, force: false}"
  ```

- **Start in off-road**: launch the manager with `mode_on_startup:=NAV2_OFFROAD`.

Confirm with `ros2 topic echo /nav2_offroad/mode_manager/status` → `current_mode:
NAV2_OFFROAD`. (If it refuses, check `~/debug` — see [DEBUGGING.md](DEBUGGING.md).)

## 3. Send an off-road goal

The off-road goal is a separate topic from the on-road mission planner goal:

```bash
ros2 topic pub --once /planning/offroad_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 30.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The bridge calls Nav2 `ComputePathToPose` + `SmoothPath`, converts the path to an
Autoware `Trajectory`, and publishes it; the manager routes it to
`/planning/trajectory`.

> There is **no RViz goal tool wired to `/planning/offroad_goal` yet** — set the
> goal with the command above, or remap the RViz "2D Goal Pose" tool to that topic
> in your RViz config. (Convenience gap, see §6.)

## 4. Engage (make the vehicle move)

Autoware must be engaged (operation mode AUTONOMOUS) for the controller to act:

- In the simulator, launch with `initial_engage_state:=true` (auto-engages), or
- engage via the AD API / HMI, or
- let the bridge do it: set `force_engage:=true` and/or `auto_accept_start:=true`
  in `nav2_path_to_trajectory_bridge.param.yaml` (defaults are `false` — leave off
  on a real vehicle unless you understand the implications).

## 5. Verify / debug

```bash
ros2 topic echo /nav2_offroad/mode_manager/status     # mode = NAV2_OFFROAD, route = offroad
ros2 topic echo /nav2_offroad/mode_manager/debug      # guard values
ros2 topic hz   /planning/trajectory                  # output flowing
```

In RViz: the `~/markers` overlay shows the mode and the green/red continuity line;
display `/nav2_offroad/planning/trajectory` and the costmap. Full guide in
[DEBUGGING.md](DEBUGGING.md). Record a run with
`ros2 launch autoware_nav2_offroad debug_record.launch.xml`.

## 6. Switch back to on-road

```bash
ros2 service call /nav2_offroad/mode_manager/change_mode \
  autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode "{target_mode: AW_PLANNING, force: false}"
```

(or the **Activate ON-ROAD** panel button). The switch only commits when the
on-road trajectory is valid and continuous with the current motion.

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
