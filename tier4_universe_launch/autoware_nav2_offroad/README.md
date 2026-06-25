# autoware_nav2_offroad

Adds Nav2-based off-road planning to Autoware. Both the standard on-road planning stack and the Nav2 planner run simultaneously. The operator switches between them at runtime using a ROS 2 service — no node restart is required.

## Architecture

```
on-road planning
  (planning_validator)
        |
        v
/planning/trajectory_pre_mux ──┐
                                ├─► trajectory_mode_manager ──► /planning/trajectory ──► trajectory_follower
/nav2_offroad/planning/trajectory ─┘          ^
        ^                                      |
nav2_path_to_trajectory_bridge          ~/change_mode service
        ^
    Nav2 planner
```

## Nodes

| Node | Executable | Description |
|------|-----------|-------------|
| `free_map_publisher` | `free_map_publisher_node` | Publishes a fully-free occupancy grid so Nav2 can plan without a sensor-based costmap (default occupancy source) |
| `perception_occupancy_relay` | `perception_occupancy_relay_node` | Relays the real Autoware perception occupancy grid into the latched Nav2 costmap input; selected with `occupancy_grid_source:=perception` |
| `nav2_path_to_trajectory_bridge` | `nav2_path_to_trajectory_bridge_node` | Calls Nav2 `ComputePathToPose` + `SmoothPath`, converts the result to an Autoware `Trajectory` |
| `trajectory_mode_manager` | `trajectory_mode_manager_node` | Owns `/planning/trajectory`; routes on-road or off-road trajectory via a guarded mode state machine with safe-stop fallback. Exposes `~/change_mode` + `~/status` (see [MODE_MANAGER_DESIGN.md](MODE_MANAGER_DESIGN.md)). Supersedes the legacy `trajectory_mode_mux_node`. |
| `controller_server` | `controller_server` (nav2_controller) | MPPI local layer (`local_layer:=mppi`, BACKLOG #11): follows the global path, emits `/cmd_vel` |
| `bt_navigator` | `bt_navigator` (nav2_bt_navigator) | NavigateToPose orchestration feeding the path to `controller_server` (`local_layer:=mppi`) |

## Switching modes at runtime

```bash
# Switch to off-road (Nav2)
ros2 service call /nav2_offroad/mode_manager/change_mode \
  autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode "{target_mode: NAV2_OFFROAD, force: false}"

# Switch back to on-road
ros2 service call /nav2_offroad/mode_manager/change_mode \
  autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode "{target_mode: AW_PLANNING, force: false}"
```

Default mode on startup: **on-road**.

To trigger off-road navigation, publish a goal on the dedicated topic (separate from the on-road mission planner goal):

```bash
ros2 topic pub --once /planning/offroad_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## Integration

There is exactly **one** unavoidable upstream patch: the planning validator must publish to `/planning/trajectory_pre_mux` so the mux can own `/planning/trajectory`. Everything else can live in this package.

### Recommended (minimal-patch) integration

Two steps total:

#### Step 1 — Apply the single required upstream patch

In `src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/planning.launch.xml`, change the `planning_validator` include arg:

```xml
<!-- before -->
<arg name="output_trajectory" value="/planning/trajectory"/>
<!-- after -->
<arg name="output_trajectory" value="/planning/trajectory_pre_mux"/>
```

This is the only edit that cannot be wrapped, because the upstream chain hardcodes the validator output topic at this point.

#### Step 2 — Launch via the wrapper

Use the wrapper launch file shipped with this package instead of the upstream `planning_simulator.launch.xml`:

```bash
ros2 launch autoware_nav2_offroad planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit
```

The wrapper:
- includes the unmodified upstream `autoware_launch/planning_simulator.launch.xml`
- adds the nav2_offroad stack (planner, bridge, mux) on top
- forwards every argument the upstream launch accepts

You do not need to add `<exec_depend>autoware_nav2_offroad</exec_depend>` to `autoware_launch/package.xml` if you are launching the wrapper from this package directly — sourcing the workspace is enough.

#### Verification

```bash
ros2 topic hz /planning/trajectory_pre_mux   # on-road planning output
ros2 topic hz /planning/trajectory            # mode manager output (same rate in on-road mode)
ros2 service list | grep change_mode          # /nav2_offroad/mode_manager/change_mode visible
```

### Optional (full launch-time mode integration)

The minimal integration above is sufficient if you only need the **runtime** switch (`set_mode` service). If you also want a launch-time selector — `navigation_mode:=nav2_offroad` that adjusts diagnostics, disables road-only nodes, and routes the simulator's occupancy grid — the following additional upstream edits are required. They are orthogonal to the mux and only affect what runs in `nav2_offroad` mode.

#### `autoware_launch/launch/autoware.launch.xml`

Add `<arg name="navigation_mode" default="autoware"/>` and the `effective_*` `<let>` blocks that swap configs when `navigation_mode==nav2_offroad`. See the diff in commit `9f61772` for the exact additions.

#### `autoware_launch/launch/components/tier4_control_component.launch.xml`

Make `vehicle_cmd_gate_param_path`, `auto_gear_cmd_topic`, `launch_lane_departure_checker`, and `launch_control_evaluator` configurable so the parent launch can override them in nav2_offroad mode.

| What changes in nav2_offroad mode | Why |
|-----------------------------------|-----|
| `vehicle_cmd_gate` uses relaxed limits | `vehicle_cmd_gate_nav2_offroad.param.yaml` ships in this package |
| `auto_gear_cmd_topic` → `/planning/gear_cmd` | The bridge publishes gear there |
| `lane_departure_checker` disabled | Requires a lanelet2 road network |
| `control_evaluator` disabled | Requires a lanelet2 road network |

#### `autoware_launch/launch/components/tier4_autoware_api_component.launch.xml`

Make `launch_routing_adaptor` configurable. The RViz routing adaptor stays
**enabled** in nav2_offroad mode: off-road uses its own dedicated "Off-road Goal"
tool (`/planning/offroad_goal`), while the standard "2D Goal Pose"
(`/planning/mission_planning/goal`) still needs the adaptor to start an on-road
route. Disabling it would silently break on-road goals while in nav2_offroad mode.

#### `autoware_launch/launch/components/tier4_simulator_component.launch.xml`

Make `occupancy_grid_map_output` configurable. In nav2_offroad mode the simulator's occupancy grid publishes to `/perception/occupancy_grid_map/simulator_map` so it does not overwrite the fully-free map that `free_map_publisher` provides on the standard topic.

#### `autoware_launch/launch/components/tier4_system_component.launch.xml`

Make `component_state_monitor_topic_path` configurable. nav2_offroad mode uses a topic list that omits road-only checks.

#### `tier4_universe_launch/tier4_*_launch/launch/*.launch.xml`

Thread the new args (`auto_gear_cmd_topic`, `launch_routing_adaptor`, `occupancy_grid_map_output`) through `control.launch.xml`, `autoware_api.launch.xml`, and `simulator.launch.xml` so the values reach the actual nodes.

#### `autoware_launch/config/system/`

Add four new config files referenced by the `effective_*` lets:

| File | Purpose |
|------|---------|
| `component_state_monitor/topics_nav2_offroad.yaml` | Topic list without road-only topics |
| `diagnostics/autoware-main-nav2-offroad.yaml` | Top-level diagnostic graph for nav2_offroad |
| `diagnostics/control-nav2-offroad.yaml` | Control diagnostics without road-only nodes |
| `diagnostics/planning-nav2-offroad.yaml` | Planning diagnostics without road-only nodes |

Without these, the diagnostic graph and component state monitor report errors for nodes intentionally absent in nav2_offroad mode.

## Configuration

See **[TUTORIAL.md](TUTORIAL.md)** for an end-to-end walkthrough (launch → switch mode → send goal → drive, plus what's still missing), **[TUNING.md](TUNING.md)** for a symptom-driven tuning guide, **[DEBUGGING.md](DEBUGGING.md)** for the observability topics (`~/status`, `~/debug`, `~/events`, `~/markers`), RViz visualization, and the rosbag record preset, and **[BACKLOG.md](BACKLOG.md)** for known follow-ups (heading-aware planner, Nav2 autostart fix, off-road validation).

Config files in `config/`:

| File | Description |
|------|-------------|
| `nav2_offroad.param.yaml` | Nav2 planner and smoother parameters |
| `nav2_path_to_trajectory_bridge.param.yaml` | Bridge topics, speeds, and timeouts |
| `free_map_publisher.param.yaml` | Free-space map resolution and frame |
| `perception_occupancy_relay.param.yaml` | Perception occupancy relay topics and `unknown_as_free` |
| `vehicle_cmd_gate_nav2_offroad.param.yaml` | Relaxed vehicle_cmd_gate limits for off-road speeds |
| `nav2_mppi_controller.param.yaml` | MPPI controller + local costmap (`local_layer:=mppi`, BACKLOG #11) |
| `nav2_bt_navigator.param.yaml` | bt_navigator NavigateToPose orchestration (`local_layer:=mppi`) |
| `nav2_smac_lattice.param.yaml` | `SmacPlannerLattice` A/B planner overlay (`global_planner:=lattice`) |

### Off-road local layer / planner selection (BACKLOG #11)

`nav2_offroad.launch.xml` exposes two selectors:

| Arg | Values | Default | Effect |
|-----|--------|---------|--------|
| `local_layer` | `bridge` \| `mppi` | `bridge` | `bridge`: path→Autoware Trajectory→`trajectory_follower` (current). `mppi`: Nav2 `controller_server` (`nav2_mppi_controller`, Ackermann) + `bt_navigator` + `local_costmap` drive the path and emit `/cmd_vel`. |
| `global_planner` | `smac_hybrid` \| `lattice` | `smac_hybrid` | `lattice` overlays `SmacPlannerLattice` (optional A/B test). |

> **`mppi` mode is bring-up only and not yet drivable end-to-end:** MPPI produces
> `/cmd_vel`, but routing it through `cmd_vel_to_control_bridge` → `vehicle_cmd_gate`
> is a separate open seam. Until that lands, use the default `local_layer:=bridge`.

Key bridge parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cruise_speed_mps` | `2.0` | Cruise speed along the Nav2 path |
| `goal_taper_distance_m` | `5.0` | Distance before goal over which speed tapers to zero |
| `goal_reached_distance_m` | `0.8` | Distance at which goal is considered reached |
| `goal_topic` | `/planning/offroad_goal` | Goal topic (separate from on-road mission planner) |
| `trajectory_topic` | `/nav2_offroad/planning/trajectory` | Output trajectory topic (fed into the mux) |

## Return-To-Home (failsafe)

The `return_home_node` adds a **free-plan shortest-path return** to a recorded home pose over the SLAM occupancy map. It reuses the same off-road single-goal autonomous flow as the rest of this package (Nav2 path planning → bridge → trajectory → vehicle_cmd_gate) and is completely independent of any Teach & Repeat work.

### What it does

1. **Record home** — the operator calls `/return_home/set_home` (or clicks the **Set Home** button in the RViz OffroadModePanel) at the desired home position. The node stores the current ego pose from `/localization/kinematic_state`.
2. **Trigger return** — `/return_home/return_home` plans a path from the current position back to the stored home and publishes a `PoseStamped` on `/planning/offroad_goal`. The existing bridge + Nav2 planner executes the path.
3. **Cancel** — `/return_home/cancel_return` publishes `Bool(true)` on `/planning/offroad_cancel` to abort the current goal; the vehicle performs a safe stop.

Status is published on `/return_home/status` (`autoware_nav2_offroad_msgs/msg/ReturnHomeState`, `result` field: `NONE=0`, `IN_PROGRESS=1`, `REACHED=2`, `CANCELED=3`, `FAILED=4`). The node reports `FAILED` if the vehicle does not reach home within `return_timeout_s` (default 120 s). RTH drives via the autonomous off-road flow (Nav2 → bridge → vehicle_cmd_gate), so the stack must already be engaged in autonomous mode for motion — publishing the goal does not itself engage the gate. Diagnostic markers are on `/return_home/markers`.

**v1 scope:** manual trigger only. An automatic comms-loss watchdog (configurable heartbeat topic + timeout → auto trigger) is planned in the backlog.

### Using the SLAM map as the costmap (`occupancy_grid_source:=slam`)

For RTH to plan over real terrain (walls, obstacles recorded during mapping), launch with:

```bash
ros2 launch autoware_nav2_offroad planning_simulator.launch.xml \
  occupancy_grid_source:=slam \
  ...
```

This routes the SLAM `/map` topic through `slam_map_relay` into the Nav2 global costmap, with `track_unknown_space:=true` so the planner only uses explored space. Without a SLAM source, the default free-space map allows planning but ignores real obstacles.

> **Note:** enabling `occupancy_grid_source:=slam` sets `track_unknown_space:=true` globally in the Nav2 global costmap. This means that for `occupancy_grid_source:=perception` unknown cells are treated as obstacles. If you want the previous behavior (unknown → free) when using perception, set `unknown_as_free:=true` in the relay's param file (`config/perception_occupancy_relay.param.yaml`).

### Services and RViz controls

| Service | Type | Description |
|---------|------|-------------|
| `/return_home/set_home` | `std_srvs/Trigger` | Record current ego pose as home |
| `/return_home/return_home` | `std_srvs/Trigger` | Trigger autonomous return to home |
| `/return_home/cancel_return` | `std_srvs/Trigger` | Cancel the current return |

All three services also appear as buttons in the **OffroadModePanel** RViz plugin.

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_return_home` | `true` | Set to `false` to skip the return_home_node |
| `return_home_param_file` | `config/return_home.param.yaml` | Node parameters (goal topic, reach tolerance, etc.) |

### Scenario script

`scripts/return_home_failsafe_scenario.py` provides a reproducible test: it records home at the current ego pose, calls `set_home`, drives the vehicle out by publishing an offset goal, waits for the vehicle to reach the out-position, calls `return_home`, and then polls `/return_home/status` until `result == REACHED` (or timeout). Final position and heading errors relative to home are printed.

```bash
python3 return_home_failsafe_scenario.py --dx 30 --dy 0 --drive-timeout 60 --timeout 120
```

Full end-to-end verification requires a running simulation with a SLAM `/map` source (not runnable headless).

## Tests

```bash
colcon test --packages-select autoware_nav2_offroad
colcon test-result --verbose
```

Two gtest suites:
- `test_trajectory_builder` — unit tests for path → trajectory conversion (3 tests)
- `test_trajectory_mode_mux` — runtime tests that spin the mux node, publish trajectories on both inputs, exercise the `set_mode` service, and verify routing (5 tests)

## Long-term goal (Option C)

Integrate OFFROAD as a third scenario inside `autoware_scenario_selector`, replacing the external mux. This would:

- give the same diagnostics, state-machine guarantees, and HMI integration as `LANEDRIVING` / `PARKING`
- remove the need for a separate mux service call
- allow automatic mode transitions based on map containment

See the plan in the package source for the required changes to `autoware_scenario_selector`.
