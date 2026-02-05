# Filtered target state from EKF
std_msgs/Header header
int32 track_id

# Position (meters, world frame)
geometry_msgs/Point position
float64[9] position_covariance  # 3x3 row-major

# Velocity (m/s, world frame)
geometry_msgs/Vector3 velocity
float64[9] velocity_covariance

# Acceleration (m/s², world frame) - estimated
geometry_msgs/Vector3 acceleration

# Track quality metrics
float64 quality_score       # 0.0 - 1.0
bool is_valid               # Track is active
int32 missed_detections     # Consecutive missed frames
