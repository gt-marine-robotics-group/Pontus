## Pool specific constants

# This is the assumed depth of the pool in m
POOL_DEPTH = 2.3

# In Cm
GRID_LINE_LENGTH = 2.7
GRID_LINE_THICKNESS = 0.30

# Image processing relating constants
# This constant is used to determine how similar two lines need to be to be considered the same
CUTOFF_THRESHOLD = 40

# Camera intrinsics
# Horizontal FOV in degrees
FOV_H = 69
# Vertical FOV in degrees
FOV_V = 55

DISPLAY_PARTICLES = True

# This is the maximum distance new particles should be from the assumed position of thesub
# This value should be no greater than half the size of a grid cell
CUTOFF_DISTANCE = GRID_LINE_LENGTH / 2.2
CUTOFF_ANGLE = 0.75

# These values are used to sample from a Gaussian distribution to simulate noise in odometry
# mu should always be 0 unless there is some drift
# sigma will determine how fast the noise grows, the larger the sigma the faster the particles disperse
TRANS_MU = 0
TRANS_SIGMA = 0.05  

ROTATE_MU = 0
ROTATE_SIGMA = 0.1
ROTATE_SAMPLE_MAX = 1

# The number of particles to use in the particle filter
NUM_PARTICLES = 50

# This is used to demonstrate the uncertainty in our marker comparison measures
# This is used for the marker likelihood function 
MARKER_DISTANCE_SIGMA = 0.3
MARKER_ANGLE_SIGMA = 0.5

# This is the number of iterations to run the particle filter before the sub starts moving to get the initial position
START_UP_ITERATIONS = 100

# Angles
NUM_START_UP_ANGLES = 4

# Grid dimenions and properties
GRID_ROWS = 31
GRID_COLUMNS = 31