std_msgs/Header header 

int32[] start_ring_index
int32[] end_ring_index

int32[]  point_col_ind # point column index in range image
float32[] point_range # point range

float32 start_orientation
float32 end_orientation
float32 orientation_diff

# Initial guess from front lidar odometry
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw
bool odom_available

# Point cloud messages
sensor_msgs/PointCloud2 cloud_projected  # original cloud
sensor_msgs/PointCloud2 extracted_corner    # extracted corner feature
sensor_msgs/PointCloud2 extracted_surface   # extracted surface feature
sensor_msgs/PointCloud2 extracted_normal   # extracted normal point(for back-end)
# sensor_msgs/PointCloud2 extracted_ground   # extracted ground

