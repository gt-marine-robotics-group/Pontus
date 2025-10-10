# ------ Gate Picture Enum -------
uint8 GATE_IMAGE_NOT_SElECTED = 0
uint8 GATE_IMAGE_FISH = 1
uint8 GATE_IMAGE_SHARK = 2

# ------ Gate Side Enum -------
uint8 GATE_NOT_COMPLETED = 0
uint8 GATE_LEFT = 1
uint8 GATE_RIGHT = 2

# ------ Slalom Side Enum ------
uint8 SLALOM_NOT_SELECTED = 0
uint8 SLALOM_LEFT = 1
uint8 SLALOM_RIGHT = 2

# ------ State ------
std_msgs/Header header

uint8 gate_picture_info 0  # Which picture did we see when passing through
uint8 gate_side_info 0  # Which side of the gate did we pass through (We get bonus points for doing opposite sides for slalom)

uint8 slalom_side_info 0  # Which side of the slalom did we choose

bool is_submerged false  # Are we submerged?
