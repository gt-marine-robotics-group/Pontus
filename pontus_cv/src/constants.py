
import numpy as np;

#   File paths to training image directories.
IMAGE_DIR_DUKE = "/Users/quinncattanach/Robosub Training Data/Duke/RoboSub 2023.v2-main-dataset-2023-04-23-12-25am.yolov4pytorch/train";
IMAGE_DIR_SONIA = "/Users/quinncattanach/Robosub Training Data/Sonia/images/train";

#   Exit codes.
EXIT_SUCCESS = 0;
EXIT_FAILURE = 1;
EXIT_FILE_NOT_FOUND = 2;

#   Default testing images.
TEST_IMAGES = [
    "./assets/0.jpg",
    "./assets/1.jpg",
    "./assets/2.jpg",
    "./assets/3.png",
    "./assets/4.png",
    "./assets/5.png",
];

#   Contour area cutoff.
AREA_CUTOFF = 100.

#   Differentiation directions.
DIFFERENTIATE_VERTICAL = 0;
DIFFERENTIATE_HORIZONTAL = 1;

DIFF_ALGO_SOBEL = 0;
DIFF_ALGO_LAPLACIAN = 1;
#   Manually constructed contour with minimal vertices.
INTEREST1_CONTOUR_SIMPLE = np.array([[0,86.2313952],[11.0551234,90.8505824],[16.5716401,77.2413834],[24.423124,66.8352269],[34.8842069,58.7640376],[46.026496,53.6803644],[89.4027953,80.9417482],[100,17.6528535],[56.4619068,9.1494176],[47.4563724,40.0789268],[30.0453142,50],[20.181257,57.9849191],[10.2795871,70.6770868]], float);

