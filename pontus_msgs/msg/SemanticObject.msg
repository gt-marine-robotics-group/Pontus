# ------ Semantic Object Types ------
uint8 GATE_IMAGE_SHARK = 0
uint8 GATE_IMAGE_FISH = 1
uint8 GATE_LEFT = 2
uint8 GATE_RIGHT = 3
uint8 SLALOM_RED = 4
uint8 SLALOM_WHITE = 5
uint8 VERTICAL_MARKER = 6
uint8 BIN = 7
uint8 OCTAGON = 8
uint8 TARGET = 9


# ------ Fields ------
std_msgs/Header header

uint8 object_type          # One of the constants above
geometry_msgs/PoseWithCovariance pose

int32 num_detections 1     # Running count (default 1)
float32 confidence          # 0.0 to 1.0

builtin_interfaces/Time last_updated   # Timestamp (ROS time)

float32 duplicant_tolerance_m 0.1      # Per-object duplicate tolerance (m)