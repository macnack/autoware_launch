# Guard inputs evaluated by trajectory_mode_manager, published on ~/debug for
# diagnosing why a mode switch was (or was not) allowed.

builtin_interfaces/Time stamp

# Which source the guards were evaluated against (true = off-road / Nav2).
bool target_is_offroad

# Source liveness (present + valid + fresh).
bool onroad_usable
bool offroad_usable
float64 onroad_age_s
float64 offroad_age_s

# Continuity of the target source's first point relative to the ego state.
float64 position_gap_m
float64 yaw_gap_rad
float64 velocity_gap_mps
bool continuity_ok

# The configured limits, echoed so a viewer can show pass/fail without the param server.
float64 max_position_gap_m
float64 max_yaw_gap_rad
float64 max_velocity_step_mps
