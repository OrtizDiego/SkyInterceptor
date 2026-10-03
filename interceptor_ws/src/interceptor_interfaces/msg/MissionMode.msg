# Current mission mode, published latched (transient local) on /mission/mode by
# mission_manager_node whenever SetMissionMode changes it
std_msgs/Header header

uint8 mode
uint8 MODE_HOLD=0
uint8 MODE_FOLLOW=1
uint8 MODE_INTERCEPT=2

bool armed          # Operator authorization to engage (INTERCEPT only)
int32 track_id      # Operator-selected track, -1 = automatic selection
