# Mode-agnostic setpoint: planners publish it on /setpoint/raw, the safety filter
# republishes the filtered command on /setpoint/safe for the trajectory controller.
# All vectors are in the world frame (ENU, header.frame_id = map).
std_msgs/Header header

# Position setpoint (m), only used when position_valid is true
geometry_msgs/Point position
bool position_valid

# Velocity setpoint or feed-forward (m/s)
geometry_msgs/Vector3 velocity

# Acceleration feed-forward (m/s^2)
geometry_msgs/Vector3 acceleration_ff

# Heading (rad, ENU) and heading rate feed-forward (rad/s)
float64 yaw
float64 yaw_rate

# Who produced the setpoint
uint8 source
uint8 SOURCE_HOLD=0
uint8 SOURCE_FOLLOW=1
uint8 SOURCE_INTERCEPT=2
uint8 SOURCE_RTL=3
