# Guidance system output
std_msgs/Header header
int32 track_id

# Acceleration command (m/s², world frame)
geometry_msgs/Vector3 acceleration_cmd

# Navigation parameters
float64 navigation_constant     # N' value
float64 closing_velocity        # V_c (m/s, negative = approaching)
float64 time_to_go              # Predicted intercept time
float64 miss_distance           # Predicted miss distance (0 = collision)

# Guidance mode
uint8 mode
uint8 MODE_PN=0
uint8 MODE_APN=1
uint8 MODE_PURSUIT=2
uint8 MODE_TERMINAL=3
uint8 MODE_IDLE=4

# Feasibility
bool is_feasible
string status_message
