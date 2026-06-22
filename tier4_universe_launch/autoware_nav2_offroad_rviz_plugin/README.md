# autoware_nav2_offroad_rviz_plugin

An RViz panel that lets an operator activate the Nav2 off-road planner or the
Autoware on-road planner at runtime.

## Usage

In RViz: **Panels → Add New Panel → autoware::nav2_offroad::rviz_plugin →
OffroadModePanel**.

The panel shows the current mode (from `/nav2_offroad/mode_manager/status`,
`autoware_nav2_offroad_msgs/msg/TrajectoryModeState`) and provides two buttons
that call `/nav2_offroad/mode_manager/change_mode`
(`autoware_nav2_offroad_msgs/srv/ChangeTrajectoryMode`):

| Button | Requested mode |
|--------|----------------|
| Activate OFF-ROAD (Nav2) | `NAV2_OFFROAD` |
| Activate ON-ROAD (Autoware) | `AW_PLANNING` |

> Requires the `trajectory_mode_manager` node (see
> [`autoware_nav2_offroad/MODE_MANAGER_DESIGN.md`](../autoware_nav2_offroad/MODE_MANAGER_DESIGN.md)).
> Until that node runs, the buttons report "service unavailable".
