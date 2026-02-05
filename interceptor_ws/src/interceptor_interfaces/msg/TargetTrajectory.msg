# For evasion system and visualization
std_msgs/Header header
int32 target_id

# Planned trajectory waypoints
geometry_msgs/Pose[] waypoints
float64[] timestamps

# Trajectory type
uint8 type
uint8 TYPE_STRAIGHT=0
uint8 TYPE_SINUSOIDAL=1
uint8 TYPE_EVASIVE=2
