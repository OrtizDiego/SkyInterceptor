# Detection from perception system
std_msgs/Header header

int32 detection_id          # Unique detection ID
int32 track_id              # Associated track ID (-1 if new)

# 2D bounding box in image coordinates
int32 bbox_x                # Top-left corner x
int32 bbox_y                # Top-left corner y
int32 bbox_width
int32 bbox_height

# Detection metadata
int32 class_id              # 0=person, 1=car, 2=truck
string class_name
float64 confidence          # 0.0 - 1.0

# 3D position in camera optical frame (meters)
geometry_msgs/Point position_camera

# 3D position in world/map frame (if available)
geometry_msgs/Point position_world
bool world_position_valid

# Depth information
float64 depth_meters        # Depth at bbox center
float64 depth_variance      # Measurement uncertainty
