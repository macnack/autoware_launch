# Operator-facing state of the return-to-home failsafe.
std_msgs/Header header

bool has_home
geometry_msgs/Pose home_pose
bool returning
float64 distance_to_home_m

# result
uint8 NONE=0
uint8 IN_PROGRESS=1
uint8 REACHED=2
uint8 CANCELED=3
uint8 result
