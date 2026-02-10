# Synchronized stereo image pair with camera calibration
std_msgs/Header header

# Left image
sensor_msgs/Image left_image

# Right image
sensor_msgs/Image right_image

# Left camera calibration
sensor_msgs/CameraInfo left_info

# Right camera calibration
sensor_msgs/CameraInfo right_info

# Synchronization metadata
float64 sync_slop_used    # Actual tolerance used for synchronization (seconds)
bool is_synced            # True if images were successfully synchronized
