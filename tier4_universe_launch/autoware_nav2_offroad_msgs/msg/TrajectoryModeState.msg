# State published by trajectory_mode_manager on ~/status (transient_local QoS).

builtin_interfaces/Time stamp

# Allowed values for current_mode / requested_mode
string MODE_STANDBY=STANDBY
string MODE_AW_PLANNING=AW_PLANNING
string MODE_NAV2_OFFROAD=NAV2_OFFROAD
string MODE_SAFE_STOP=SAFE_STOP

# Allowed values for transition
string TRANSITION_NONE=NONE
string TRANSITION_TO_AW=TRANSITION_TO_AW
string TRANSITION_TO_NAV2=TRANSITION_TO_NAV2

# Authoritative current mode (one of MODE_*)
string current_mode

# Operator/policy requested target mode (one of MODE_*); equals current_mode when settled
string requested_mode

# Active transition (one of TRANSITION_*)
string transition

# Which input trajectory is currently routed to /planning/trajectory (e.g. "onroad", "offroad", "safe_stop")
string active_planner

# Non-empty when current_mode == MODE_SAFE_STOP or when a transition was aborted
string fault_reason
