## Pool specific constants

# This is the assumed depth of the pool in cm
DEPTH = 500

# In Cm
GRID_LINE_LENGTH = 276
GRID_LINE_THICKNESS = 30



# This is the maximum distance new particles should be from the assumed position of thesub
# This value should be no greater than half the size of a grid cell
CUTOFF_DISTANCE = 5.0

# These values are used to sample from a Gaussian distribution to simulate noise in odometry
TRANS_MU = 0
TRANS_SIGMA = 0.5

ROTATE_MU = 0
ROTATE_SIGMA = 0.2
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
GRID_ROWS = 30
GRID_COLUMNS = 30

## Camera intrinsics

# In Degrees
HORIZONTAL_FOV = 60
VERTICAL_FOV = 40 