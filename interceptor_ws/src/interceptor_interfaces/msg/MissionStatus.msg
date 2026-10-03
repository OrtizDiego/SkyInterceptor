# Mission telemetry, published on /mission/status by safety_filter_node
std_msgs/Header header

# Mission mode (MissionMode constants)
uint8 mode

# Phase of the active planner
uint8 phase
uint8 PHASE_IDLE=0
# FOLLOW
uint8 PHASE_TRACKING=1
uint8 PHASE_LOST_HOLD=2
uint8 PHASE_RTL=3
# INTERCEPT
uint8 PHASE_SEARCH=10
uint8 PHASE_PURSUE=11
uint8 PHASE_CAPTURE=12
uint8 PHASE_DONE=13
uint8 PHASE_ABORT=14

int32 target_track_id       # -1 = no target
float64 distance_to_target  # m, NaN when there is no target

# True whenever the safety filter modified the planner's setpoint
bool safety_active

# Why the safety filter intervened or the planner holds (empty when nominal)
string reason
