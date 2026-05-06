# autoware_nav2_offroad

Nav2-based off-road planning bridge for Autoware. Converts a goal pose into an `autoware_planning_msgs/Trajectory` by delegating path planning to Nav2's grid planner + smoother, so a vehicle can leave the HD-map / lanelet2 road network and continue under Autoware's existing controller.

## Current scope (Option A — runtime trajectory mux)

The near-term integration runs **both** trajectory producers in parallel and switches the controller's input at runtime, without restarting any node and without a trajectory-publication gap:

```
on-road planning (tier4_planning_component) ─► /planning/scenario_planning/trajectory ─┐
                                                                                       ├─► trajectory_mode_mux ─► /planning/trajectory ─► trajectory_follower
nav2_path_to_trajectory_bridge              ─► /nav2_offroad/planning/trajectory     ──┘
```

The mux replaces the temporary `trajectory_relay` in `autoware.launch.xml` (the one that today bridges `/planning/trajectory` ↔ `/planning/scenario_planning/trajectory` for the topic-rename migration). To make the diagram above real, the on-road planning output is left on its scenario-planning name and the Nav2 bridge is namespaced to `/nav2_offroad/planning/trajectory`, so the two producers never collide on `/planning/trajectory`.

- The on-road Autoware planning stack stays running on its normal trajectory pipeline.
- The Nav2 stack (`planner_server`, `smoother_server`, `nav2_path_to_trajectory_bridge`) runs in parallel; it idles cheaply while no offroad goal is active and can optionally be lifecycle-managed (`configure` only) until the operator switches modes.
- A small mux node arbitrates which trajectory reaches `/planning/trajectory`, plus the side channels (`/planning/gear_cmd`, `/planning/turn_indicators_cmd`, `/planning/hazard_lights_cmd`, `/autoware/engage`).
- Mode switch is a single service call, e.g. with the standard `topic_tools/srv/MuxSelect`:
  ```
  ros2 service call /planning/mode_mux/select topic_tools/srv/MuxSelect \
      "{topic: /nav2_offroad/planning/trajectory}"
  ```
- `force_engage` and `auto_accept_start` in `nav2_path_to_trajectory_bridge` are gated on the mux state so the bridge does not engage the vehicle while on-road planning owns the trajectory.
- The `free_map_publisher` is **dev-only**. On a real vehicle the costmap input comes from the live `/perception/occupancy_grid_map/map`; the free-map node is used only for planning_simulator scenarios where no real perception is available.

### Side-channel QoS caveat

The bridge currently publishes `TurnIndicatorsCommand`, `HazardLightsCommand`, and `GearCommand` with `rclcpp::QoS{1}.transient_local()` (see `nav2_path_to_trajectory_bridge_node.cpp:127-134`). Stock `topic_tools/MuxNode` does not preserve transient-local durability across the mux — late subscribers downstream of the mux would not receive the latched value. Two practical options:

- a custom mux node (or per-topic relay nodes wrapped around an external mode flag) that re-publishes with the same `transient_local` QoS profile each side channel needs, or
- demote the bridge's side-channel publishers to plain `reliable` and accept that downstream consumers must already be subscribed when the bridge sends.

The single trajectory mux is fine with `topic_tools/MuxNode` because `/planning/trajectory` is published with default volatile QoS today.

## Long-term goal — Option C: scenario-arbiter integration

The longer-term direction is to fold this into `autoware_scenario_selector` so the runtime mux and the `navigation_mode` launch arg can be removed. Off-road becomes a **third scenario** alongside `LANEDRIVING` and `PARKING`. Autoware planning, Nav2, and the bridge are all always launched; `scenario_selector` arbitrates which one currently owns `/planning/scenario_planning/trajectory`.

### Why scenario_selector

- It already loads the lanelet2 map and answers "am I on the road network?" via `isInLane`, `isAlongLane`, and `isInParkingLot` (`autoware_universe/planning/autoware_scenario_selector/src/node.cpp`). The same code that decides `LANEDRIVING ↔ PARKING` can decide `LANEDRIVING ↔ OFFROAD`.
- Exactly one publisher owns `/planning/scenario_planning/trajectory` at a time — no mux, no race, no dual `vehicle_cmd_gate` configurations.
- The existing transition machinery (stopped-vehicle requirement, autonomous-mode requirement, dwell timers, route-UUID reset) is reused; only the offroad predicate is new.
- `~output/scenario` becomes the single source of truth that downstream components (Nav2 lifecycle_manager, the bridge's engage suppression, HMI, diagnostics) listen to.

### Required changes (target design)

1. **Add scenario constant.** Extend `autoware_internal_planning_msgs/msg/Scenario.msg` with `string OFFROAD=Offroad`.
2. **Bridge becomes a scenario producer.**
   - Re-target the trajectory output from `/planning/trajectory` to `/planning/scenario_planning/offroad/trajectory`.
   - Remove the `Engage` and `AcceptStart` publishers entirely — the standard Autoware ADAPI path handles them once the vehicle is shared.
   - Keep the `GearCommand` and turn/hazard indicator publishers, but gate them on `~output/scenario == OFFROAD` so they are silent while another scenario owns the vehicle.
3. **Third input in scenario_selector.** Add `sub_offroad_trajectory_`, `offroad_trajectory_`, `onOffroadTrajectory`, and `isCurrentOffroad`; extend `getScenarioTrajectory` to return the offroad trajectory; add the `input/offroad/trajectory` remap in `scenario_selector.launch.xml` and in the `tier4_planning_component` wrapper.
4. **Transition policy** (in `updateCurrentScenario`). Pick one and tune dwell times to match the existing `lane_stopping_timeout_s` / `empty_parking_trajectory_timeout_s` style:
   - **4a — Manual request** (first milestone): a `~/input/offroad_request` topic / service set by the operator HMI. `LANEDRIVING → OFFROAD` requires `is_stopped && isAutonomous() && requested_offroad`; the reverse requires `!requested_offroad && is_stopped && isAlongLane(...)`.
   - **4b — Automatic by lanelet containment**: drive transitions from `!isInLane(map, current_pose)` with a 2–3 s dwell timer to prevent flapping at the road edge. Same hysteresis pattern already used for parking.
   - **4c — Goal-driven**: classify the goal pose at route-reception (in lanelet → LANEDRIVING, in parking polygon → PARKING, else → OFFROAD). Mirrors how PARKING is selected today; needs an offroad-aware route or a parallel goal topic since `mission_planner` rejects off-network goals.
5. **Lifecycle Nav2 by scenario.** A small supervisor (or extension of `nav2_lifecycle_manager`) watches `~output/scenario`: `configure → activate` Nav2 when `OFFROAD` activates, `deactivate` otherwise. Keeps the global costmap from running when not needed.
6. **Remove `navigation_mode` from the launchers.** All the `effective_*` `<let>` blocks in `autoware.launch.xml` and `planning_simulator.launch.xml` become removable once scenario_selector arbitrates. The wins this delivers:
   - `effective_launch_planning`, `effective_launch_control`, `effective_launch_routing_adaptor`, `effective_launch_lane_departure_checker`, `effective_launch_control_evaluator` — the planning/control/API gates collapse, since Autoware planning is always up.
   - `effective_vehicle_cmd_gate_param_path` — one `vehicle_cmd_gate` config (the on-road one) covers both modes; offroad cruise speed of 2 m/s fits inside on-road limits. Keep the offroad config file only if field testing shows divergent limits are actually needed.
   - `effective_auto_gear_cmd_topic` — remove; the (now scenario-gated) bridge gear publisher uses the same `/planning/gear_cmd` as the rest of planning.
   - `effective_component_state_monitor_topic_path` and `effective_diagnostic_graph_aggregator_graph_path` — single component-state and diagnostic graph configs; differences move into the existing graph as scenario-conditional nodes if needed.
   - `simulator_occupancy_grid_map_topic` (in `planning_simulator.launch.xml`) — disappears with the dev-only `free_map_publisher`.
7. **Tests.** Extend `autoware_scenario_selector/test/test_autoware_scenario_selector_node_interface.cpp` with cases for `LANEDRIVING ↔ OFFROAD` transitions (off-network odometry → OFFROAD after dwell; return-to-lane → LANEDRIVING after dwell + along-lane check) and confirm no direct `OFFROAD ↔ PARKING` transition.

### Migration plan

1. Land Option A (runtime mux) with the dev-only `free_map_publisher` swapped out for the live perception costmap on the real vehicle.
2. Add the `OFFROAD` scenario constant, bridge re-targeting, and the third input in `scenario_selector` — at this point both the mux and the scenario arbiter coexist; the mux can simply select the scenario_selector output.
3. Implement transition policy 4a (manual) and validate on-vehicle.
4. Remove the runtime mux and the `navigation_mode` launch arg; scenario_selector becomes the sole arbiter.
5. Optionally graduate the policy to 4b or 4c once transition criteria are tuned in the field.

### Open questions

- **Operator HMI for 4a.** Where does `offroad_request` come from? An RViz panel? An ADAPI service exposed through `web_auto`? This shapes whether `scenario_selector` exposes a topic or a service for the request.
- **Goal source.** When the user is on-road with an active `LaneletRoute` and wants to detour offroad, does the bridge consume `/planning/mission_planning/goal` directly (bypassing `mission_planner`), or do we add a parallel `/planning/offroad_goal` topic so the on-road route stays armed for re-entry?
- **Re-entry semantics.** When `OFFROAD → LANEDRIVING` fires, does the on-road `LaneletRoute` get re-evaluated from the new pose, or does the operator have to set a fresh route? `route_handler` already exposes the bits needed; behavior just has to be specified.
- **Diagnostics during the OFFROAD scenario.** Several on-road diagnostic checks (lane departure, planning evaluator, behavior path planner heartbeats) are currently silenced via the `nav2_offroad` graph file. With scenario_selector in the loop, these become scenario-conditional checks in a single graph; the exact set worth keeping vs. dropping needs a deliberate review.

## Package contents (current)

- `nav2_path_to_trajectory_bridge_node` — receives `PoseStamped` goal + odometry, sends `ComputePathToPose` + `SmoothPath` actions to Nav2, publishes `autoware_planning_msgs/Trajectory` plus auxiliary commands.
- `free_map_publisher_node` — dev-only flat occupancy grid for planning_simulator usage.
- `trajectory_builder` library — resamples the Nav2 path, rebuilds yaw from `atan2(dy, dx)`, applies a linear velocity taper toward the goal, pads to ≥3 points, integrates `time_from_start`.
- `test/test_trajectory_builder.cpp` — unit tests for `trajectory_builder` covering stop trajectories, forward trajectories with goal taper, and degenerate (single-pose) input. Add cases here when changing the velocity profile or yaw reconstruction.
- `launch/nav2_offroad.launch.xml` — brings up the free map publisher, `nav2_planner`, `nav2_smoother`, `nav2_lifecycle_manager`, and the bridge.
- `config/` — Nav2 planner/smoother/costmap params, bridge params, dev free-map params, and an offroad-tuned `vehicle_cmd_gate` parameter file.
