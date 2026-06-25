# Packages changed to integrate `autoware_nav2_offroad`

This file records every package and file that was modified outside of
`autoware_nav2_offroad` itself in order to wire the off-road (Nav2) stack into
Autoware. It is the integration counterpart to the package `README.md`.

All of these packages live in the **`autoware_launch`** repository
(`https://github.com/autowarefoundation/autoware_launch`). No package in any
other Autoware repository needs to be modified — the Nav2 runtime packages
(`nav2_planner`, `nav2_smoother`, `nav2_navfn_planner`, `nav2_lifecycle_manager`)
are pulled in automatically by `rosdep` from this package's `package.xml`.

Integration landed in commit `77b484a`
(`feat(autoware_nav2_offroad): restore launch-time integration and complete packaging`),
plus one earlier required patch noted below.

## Summary

| Package | Change | Required? |
|---------|--------|-----------|
| `tier4_planning_launch` | route `planning_validator` output to `/planning/trajectory_pre_mux` so the mux can own `/planning/trajectory` | **Required** (minimal integration) |
| `autoware_launch` | `navigation_mode:=nav2_offroad` selector, per-mode configs, `exec_depend` | Optional (launch-time mode) |
| `tier4_control_launch` | make the auto gear-cmd topic configurable | Optional (launch-time mode) |
| `tier4_autoware_api_launch` | make the routing adaptor toggle configurable | Optional (launch-time mode) |
| `tier4_simulator_launch` | make the occupancy-grid output topic configurable | Optional (launch-time mode) |

> The **runtime** switch (the `trajectory_mode_mux` `set_mode` service + the
> package's own `planning_simulator.launch.xml` wrapper) needs **only** the
> `tier4_planning_launch` patch. The remaining packages are touched only for the
> optional launch-time `navigation_mode` selector.

## Per-package detail

### `tier4_planning_launch` — the one required patch

- `launch/planning.launch.xml`
  - `planning_validator` `output_trajectory`: `/planning/trajectory` → `/planning/trajectory_pre_mux`
  - This is the only edit that cannot be wrapped: the upstream chain hardcodes
    the validator output topic here. The `trajectory_mode_mux` then owns
    `/planning/trajectory`.

### `autoware_launch`

- `launch/autoware.launch.xml`
  - add `navigation_mode` arg (`autoware` | `nav2_offroad`)
  - add `effective_*` `<let>` blocks that, when `navigation_mode==nav2_offroad`,
    swap in: the nav2_offroad diagnostics graph, the nav2_offroad
    component_state_monitor topics, the relaxed `vehicle_cmd_gate` params, and
    disable the road-only nodes (routing adaptor, lane_departure_checker,
    control_evaluator)
- `launch/planning_simulator.launch.xml`
  - accept `navigation_mode`; include the nav2_offroad stack when selected
- `launch/components/tier4_control_component.launch.xml`
  - make `vehicle_cmd_gate_param_path`, `auto_gear_cmd_topic`,
    `launch_lane_departure_checker`, `launch_control_evaluator` configurable
- `launch/components/tier4_autoware_api_component.launch.xml`
  - forward `launch_routing_adaptor`
- `launch/components/tier4_simulator_component.launch.xml`
  - forward `occupancy_grid_map_output`
- `launch/components/tier4_system_component.launch.xml`
  - make `component_state_monitor_topic_path` configurable
- `package.xml`
  - add `<exec_depend>autoware_nav2_offroad</exec_depend>`
- `config/system/diagnostics/autoware-main-nav2-offroad.yaml` *(new)*
  - top-level diagnostic graph for nav2_offroad mode
- `config/system/diagnostics/control-nav2-offroad.yaml` *(new)*
  - control diagnostics without the road-only checks (lane_departure)
- `config/system/diagnostics/planning-nav2-offroad.yaml` *(new)*
  - planning diagnostics without the road-only checks (routing/mission route)
- `config/system/component_state_monitor/topics_nav2_offroad.yaml` *(new)*
  - component-state-monitor topic list without the road-only topics

### `tier4_control_launch`

- `launch/control.launch.xml`
  - add arg `auto_gear_cmd_topic` (default `/control/shift_decider/gear_cmd`)
  - remap `input/auto/gear_cmd` → `$(var auto_gear_cmd_topic)` so nav2_offroad
    mode can source the gear command from the bridge

### `tier4_autoware_api_launch`

- `launch/autoware_api.launch.xml`
  - add arg `launch_routing_adaptor` (default `true`)
  - forward it into `rviz_adaptors.launch.xml` so the routing adaptor can be
    disabled in nav2_offroad mode (goals go to `/planning/offroad_goal` instead)

### `tier4_simulator_launch`

- `launch/simulator.launch.xml`
  - add arg `occupancy_grid_map_output` (default `/perception/occupancy_grid_map/map`)
  - use `$(var occupancy_grid_map_output)` for the simulator occupancy grid so
    nav2_offroad mode can redirect it and leave the `free_map_publisher` map on
    the standard topic

## Not a code change: dependency provisioning

The Nav2 packages are declared in `autoware_nav2_offroad/package.xml` and are
resolved automatically wherever the workspace runs `rosdep install --from-paths
src` (Docker image build via `docker/Dockerfile`, and `setup-dev-env.sh`). They
are **not** pinned in `repositories/autoware.repos` and require no entry in
`ansible/` or `docker/` — provided this package is present in the source tree at
dependency-resolution time.
