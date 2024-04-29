## Pool specific constants

# This is the assumed depth of the pool in m
POOL_DEPTH = 2.3

# In Cm
GRID_LINE_LENGTH = 2.7
GRID_LINE_THICKNESS = 0.30

# Image processing relating constants
# This constant is used to determine how similar two lines need to be to be considered the same
CUTOFF_THRESHOLD = 40

# This is used to determine the direction of the gradient from the line
# If set to true, that means the line has a greater light intensity compared to the background
GRADIENT_POINTS_INTO_LINE = False

# Camera intrinsics
# Horizontal FOV in degrees
FOV_H = 69
# Vertical FOV in degrees
FOV_V = 55

# This is the distance from the depth sensor to the camera in m
CAMERA_DEPTH_OFFSET = 0.15

DISPLAY_PARTICLES = True

# This is the maximum distance new particles should be from the assumed position of thesub
# This value should be no greater than half the size of a grid cell
CUTOFF_DISTANCE = GRID_LINE_LENGTH / 2.2
CUTOFF_ANGLE = 0.75

# These values are used to sample from a Gaussian distribution to simulate noise in odometry
# mu should always be 0 unless there is some drift
# sigma will determine how fast the noise grows, the larger the sigma the faster the particles disperse
TRANS_MU = 0
TRANS_SIGMA = 0.07  

ROTATE_MU = 0
ROTATE_SIGMA = 0.1
ROTATE_SAMPLE_MAX = 1

# The number of particles to use in the particle filter
NUM_PARTICLES = 150

# This is used to demonstrate the uncertainty in our marker comparison measures
# This is used for the marker likelihood function 
# A higher value means that particles further away from the marker will have a higher density
MARKER_DISTANCE_SIGMA = 0.2
MARKER_ANGLE_SIGMA = 0.3

# This is the number of iterations to run the particle filter before the sub starts moving to get the initial position
START_UP_ITERATIONS = 25

# Angles
NUM_START_UP_ANGLES = 1

# Grid dimenions and properties
GRID_ROWS = 31
GRID_COLUMNS = 31